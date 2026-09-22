"""Visual conversion must keep material bindings, placement and mirrored URDF scales."""
import sys
import tempfile
import unittest
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from openarm_visuals import collada_parts, prepare_visuals, reduce_detail

DAE = '''<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema">
<library_effects><effect id="dark"><profile_COMMON><technique><lambert><diffuse><color>0.1 0.2 0.3 1</color></diffuse></lambert></technique></profile_COMMON></effect></library_effects>
<library_materials><material id="paint"><instance_effect url="#dark"/></material></library_materials>
<library_geometries><geometry id="part"><mesh>
<source id="positions"><float_array>0 0 0 1 0 0 0 1 0</float_array><technique_common><accessor count="3" stride="3"/></technique_common></source>
<source id="normals"><float_array>0 0 1</float_array><technique_common><accessor count="1" stride="3"/></technique_common></source>
<vertices id="v"><input semantic="POSITION" source="#positions"/></vertices>
<triangles material="symbol" count="1"><input semantic="VERTEX" source="#v" offset="0"/><input semantic="NORMAL" source="#normals" offset="1"/><p>0 0 1 0 2 0</p></triangles>
</mesh></geometry></library_geometries>
<library_visual_scenes><visual_scene id="scene"><node><matrix>1 0 0 5 0 1 0 6 0 0 1 7 0 0 0 1</matrix><instance_geometry url="#part"><bind_material><technique_common><instance_material symbol="symbol" target="#paint"/></technique_common></bind_material></instance_geometry></node></visual_scene></library_visual_scenes>
<scene><instance_visual_scene url="#scene"/></scene></COLLADA>'''

class Visuals(unittest.TestCase):
    def test_colour_transform_normals_and_mirror(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            dae = folder / 'part.dae'; dae.write_text(DAE)
            points, faces, normals, normal_faces, color = next(collada_parts(dae))
            np.testing.assert_allclose(points[0], [5, 6, 7])
            np.testing.assert_allclose(normals, [[0, 0, 1]])
            self.assertEqual(color, (0.1, 0.2, 0.3, 1))
            urdf = ET.fromstring(f'<robot><link name="left"><visual name="visual"><origin xyz="1 2 3"/><geometry><mesh filename="{dae}" scale="0.001 -0.001 0.001"/></geometry></visual><collision name="keep"/></link></robot>')
            prepare_visuals(urdf, folder)
            visual = urdf.find('link/visual')
            self.assertEqual(visual.find('geometry/mesh').get('scale'), '0.001 -0.001 0.001')
            self.assertEqual(visual.find('origin').get('xyz'), '1 2 3')
            self.assertEqual(visual.find('material/color').get('rgba'), '0.1 0.2 0.3 1.0')
            self.assertIsNotNone(urdf.find('link/collision'))
            self.assertIn('f 1//1 2//1 3//1', Path(visual.find('geometry/mesh').get('filename')).read_text())

    def test_unsupported_geometry_fails_explicitly(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'part.dae'
            path.write_text(DAE.replace('triangles', 'polylist'))
            with self.assertRaisesRegex(ValueError, 'Unsupported COLLADA primitive'):
                list(collada_parts(path))

    def test_detail_reduction_keeps_valid_triangles_and_normals(self):
        x, y = np.meshgrid(np.arange(80), np.arange(80))
        points = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size))) * 0.1
        faces = []
        for row in range(79):
            for col in range(79):
                a = row * 80 + col
                faces.extend(([a, a+1, a+80], [a+1, a+81, a+80]))
        faces = np.array(faces)
        p, f, n, nf = reduce_detail(points, faces, np.array([[0.,0.,1.]]), np.zeros_like(faces), 0.5)
        self.assertLess(len(f), len(faces)//2)
        self.assertTrue(np.all(f >= 0) and np.all(f < len(p)))
        self.assertTrue(np.all(np.linalg.norm(np.cross(p[f[:,1]]-p[f[:,0]], p[f[:,2]]-p[f[:,0]]),axis=1)>0))
        np.testing.assert_allclose(n, [[0,0,1]])
        self.assertEqual(nf.shape, f.shape)

if __name__ == '__main__': unittest.main()
