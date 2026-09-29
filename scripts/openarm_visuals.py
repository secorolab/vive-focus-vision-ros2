"""Convert OpenArm's coloured COLLADA visuals to material-separated OBJ meshes.

MuJoCo cannot read DAE. Preserve the authored triangles, normals, node transforms and
flat diffuse colours, with 1 mm clustering for dense CAD surfaces; never substitute collision hulls for the visual geometry.
The supported subset is deliberately explicit: unsupported assets fail rather than losing
materials silently. Generated meshes stay beside the generated robot, outside the repo.
"""
import copy
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

NS = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}


def collada_parts(path):
    root = ET.parse(path).getroot()
    axis = root.find('c:asset/c:up_axis', NS)
    unit = root.find('c:asset/c:unit', NS)
    if (axis is not None and axis.text.strip() != 'Z_UP') or (unit is not None and float(unit.get('meter', '1')) != 1):
        raise ValueError(f'Expected OpenArm Z-up source coordinates with unit=1: {path}')
    if root.findall('.//c:diffuse/c:texture', NS):
        raise ValueError(f'Textured COLLADA is not supported by the OpenArm converter: {path}')
    effects = {}
    for effect in root.findall('./c:library_effects/c:effect', NS):
        color = effect.find('.//c:diffuse/c:color', NS)
        if color is None:
            raise ValueError(f'Missing diffuse colour in {path}: {effect.get("id")}')
        effects[effect.get('id')] = tuple(map(float, color.text.split()))
    materials = {m.get('id'): effects[m.find('c:instance_effect', NS).get('url')[1:]]
                 for m in root.findall('./c:library_materials/c:material', NS)}
    geometry = {g.get('id'): g.find('c:mesh', NS)
                for g in root.findall('./c:library_geometries/c:geometry', NS)}
    scene_id = root.find('./c:scene/c:instance_visual_scene', NS).get('url')[1:]
    scene = next(s for s in root.findall('./c:library_visual_scenes/c:visual_scene', NS)
                 if s.get('id') == scene_id)

    def source(mesh, reference):
        src = next(s for s in mesh.findall('c:source', NS) if s.get('id') == reference[1:])
        accessor = src.find('c:technique_common/c:accessor', NS)
        data = np.fromstring(src.find('c:float_array', NS).text, sep=' ')
        stride = int(accessor.get('stride', '1'))
        offset = int(accessor.get('offset', '0'))
        count = int(accessor.get('count'))
        if stride < 3:
            raise ValueError(f'Expected 3D coordinates in {path}')
        return data[offset:offset + count * stride].reshape(count, stride)[:, :3]

    def nodes(parent, transform):
        for node in parent.findall('c:node', NS):
            local = np.eye(4)
            for child in node:
                kind = child.tag.split('}')[-1]
                if kind == 'matrix':
                    local = local @ np.fromstring(child.text, sep=' ').reshape(4, 4)
                elif kind in ('translate', 'rotate', 'scale', 'lookat', 'skew', 'instance_node'):
                    raise ValueError(f'Unsupported COLLADA transform {kind} in {path}')
            world = transform @ local
            for instance in node.findall('c:instance_geometry', NS):
                mesh = geometry[instance.get('url')[1:]]
                bindings = {m.get('symbol'): materials[m.get('target')[1:]]
                            for m in instance.findall('.//c:instance_material', NS)}
                for primitive in mesh:
                    kind = primitive.tag.split('}')[-1]
                    if kind in ('source', 'vertices', 'extra'):
                        continue
                    if kind != 'triangles':
                        raise ValueError(f'Unsupported COLLADA primitive {kind} in {path}')
                    inputs = primitive.findall('c:input', NS)
                    width = max(int(i.get('offset', '0')) for i in inputs) + 1
                    indexes = np.fromstring(primitive.find('c:p', NS).text, dtype=int, sep=' ').reshape(-1, width)
                    vertex = next(i for i in inputs if i.get('semantic') == 'VERTEX')
                    vertices = next(v for v in mesh.findall('c:vertices', NS)
                                    if v.get('id') == vertex.get('source')[1:])
                    position = next(i for i in vertices.findall('c:input', NS) if i.get('semantic') == 'POSITION')
                    points = source(mesh, position.get('source'))
                    points = points @ world[:3, :3].T + world[:3, 3]
                    faces = indexes[:, int(vertex.get('offset', '0'))].reshape(-1, 3)
                    normal = next((i for i in inputs if i.get('semantic') == 'NORMAL'), None)
                    normals = normal_faces = None
                    if normal is not None:
                        normals = source(mesh, normal.get('source')) @ np.linalg.inv(world[:3, :3])
                        normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
                        normal_faces = indexes[:, int(normal.get('offset', '0'))].reshape(-1, 3)
                    if np.linalg.det(world[:3, :3]) < 0:
                        faces = faces[:, ::-1]
                        if normal_faces is not None:
                            normal_faces = normal_faces[:, ::-1]
                    if not np.isfinite(points).all() or (normals is not None and not np.isfinite(normals).all()):
                        raise ValueError(f'Invalid visual coordinates in {path}')
                    symbol = primitive.get('material')
                    # A few source submeshes have no material assignment at all.
                    color = bindings[symbol] if symbol else (0.7, 0.7, 0.7, 1.0)
                    yield points, faces, normals, normal_faces, color
            yield from nodes(node, world)
    yield from nodes(scene, np.eye(4))


def reduce_detail(points, faces, normals, normal_faces, cell):
    """Merge fine CAD tessellation within each material, retaining face normals."""
    if len(faces) < 4000:
        return points, faces, normals, normal_faces
    _, inverse = np.unique(np.floor(points / cell).astype(np.int64), axis=0, return_inverse=True)
    counts = np.bincount(inverse)
    merged = np.column_stack([np.bincount(inverse, weights=points[:, axis]) / counts for axis in range(3)])
    remapped = inverse[faces]
    valid = (remapped[:, 0] != remapped[:, 1]) & (remapped[:, 1] != remapped[:, 2]) & (remapped[:, 0] != remapped[:, 2])
    selected = np.flatnonzero(valid)
    if not len(selected):
        return points, faces, normals, normal_faces
    # Eliminate duplicate triangles while keeping the winding of the first occurrence.
    _, unique = np.unique(np.sort(remapped[selected], axis=1), axis=0, return_index=True)
    selected = selected[np.sort(unique)]
    faces = remapped[selected]
    used, mapped = np.unique(faces, return_inverse=True)
    points, faces = merged[used], mapped.reshape(-1, 3)
    if normal_faces is not None:
        normal_faces = normal_faces[selected]
        used, mapped = np.unique(normal_faces, return_inverse=True)
        normals, normal_faces = normals[used], mapped.reshape(-1, 3)
    return points, faces, normals, normal_faces


def hardware_colour(path, color):
    """Approximate the black V1 hardware finish, not the pale CAD display palette.

    Scoped to the known V1 mesh names; source files and unknown meshes stay intact.
    Values are deliberately dark enough for the headset's bright ambient lighting.
    """
    name = path.stem
    rgb = np.array(color[:3])
    if name == 'body_link0':
        # Central housing and extrusion are dark; base/braces and hardware silver.
        value = (0.025, 0.029, 0.035) if rgb.mean() < 0.3 else (0.46, 0.48, 0.50)
    elif name in {f'link{i}' for i in range(8)}:
        if np.allclose(rgb, (0.62745,) * 3, atol=1e-4) or (
                name == 'link7' and np.allclose(rgb, (0.79608,) * 3, atol=1e-4)):
            value = (0.48, 0.50, 0.52)
        elif rgb.max() < 0.01:
            value = (0.009, 0.011, 0.014)
        elif rgb.mean() < 0.2:
            value = (0.018, 0.021, 0.026)
        else:
            value = (0.035, 0.040, 0.047)
    else:
        return color
    return (*value, color[3])


def prepare_visuals(urdf, folder, appearance='cad'):
    """Replace each DAE visual by OBJ parts, retaining origin, scale and per-part colour."""
    destination = folder / 'visual_meshes'
    destination.mkdir(parents=True, exist_ok=True)
    cache = {}
    for link in urdf.findall('link'):
        for visual in list(link.findall('visual')):
            mesh = visual.find('geometry/mesh')
            if mesh is None or Path(mesh.get('filename')).suffix.lower() != '.dae':
                continue
            path = Path(mesh.get('filename'))
            scale = max(abs(float(x)) for x in mesh.get('scale', '1 1 1').split())
            cache_key = (path, scale)
            if cache_key not in cache:
                key = hashlib.sha256(f'{path}:{scale}'.encode()).hexdigest()[:10]
                parts = []
                for index, (points, faces, normals, normal_faces, color) in enumerate(collada_parts(path)):
                    if appearance == 'hardware':
                        color = hardware_colour(path, color)
                    points, faces, normals, normal_faces = reduce_detail(
                        points, faces, normals, normal_faces, 0.001 / scale)
                    output = destination / f'{path.stem}_{key}_{index}.obj'
                    with output.open('w') as stream:
                        for point in points:
                            stream.write('v ' + ' '.join(f'{x:.9g}' for x in point) + '\n')
                        if normals is not None:
                            for normal in normals:
                                stream.write('vn ' + ' '.join(f'{x:.9g}' for x in normal) + '\n')
                        for i, face in enumerate(faces):
                            entries = [str(int(v) + 1) if normal_faces is None else
                                       f'{int(v)+1}//{int(normal_faces[i,j])+1}' for j, v in enumerate(face)]
                            stream.write('f ' + ' '.join(entries) + '\n')
                    parts.append((output, color))
                if not parts:
                    raise ValueError(f'No visual triangles found in {path}')
                cache[cache_key] = parts
            link.remove(visual)
            for index, (output_path, color) in enumerate(cache[cache_key]):
                part = copy.deepcopy(visual)
                name = visual.get('name', link.get('name') + '_visual') + f'_part{index}'
                part.set('name', name)
                part.find('geometry/mesh').set('filename', str(output_path))
                old = part.find('material')
                if old is not None:
                    part.remove(old)
                material = ET.SubElement(part, 'material', name=name + '_material')
                ET.SubElement(material, 'color', rgba=' '.join(map(str, color)))
                link.append(part)
    print(f'Prepared {sum(len(parts) for parts in cache.values())} coloured visual mesh parts')
