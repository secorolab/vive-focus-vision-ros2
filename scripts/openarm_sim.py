#!/usr/bin/env python3
"""Prepare, export and launch the OpenArm V1 motion-test simulation.

Requires sourced ROS Jazzy and vive_vr_ros2/OpenArm workspaces. No hardware control.
See docs/openarm_sim.md. Generated files are disposable; source descriptions are untouched.
"""

import argparse
import copy
import ctypes as C
import json
import os
from pathlib import Path
import shlex
import signal
import socket
import subprocess
import sys
import xml.etree.ElementTree as ET


REPO = Path(__file__).resolve().parents[1]


def run(*args):
    command = [str(arg) for arg in args]
    print('+ ' + shlex.join(command), flush=True)
    subprocess.run(command, check=True)


def write_xml(root, path):
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


class Mujoco:
    """Use the wrapper's existing library, without installing a second MuJoCo."""

    def __init__(self, library):
        self.lib = C.CDLL(str(library))
        for name, args, result in [
            ('mj_parseXML', [C.c_char_p, C.c_void_p, C.c_char_p, C.c_int], C.c_void_p),
            ('mj_compile', [C.c_void_p, C.c_void_p], C.c_void_p),
            ('mjs_getError', [C.c_void_p], C.c_char_p),
            ('mj_saveXML', [C.c_void_p, C.c_char_p, C.c_char_p, C.c_int], C.c_int),
            ('mj_deleteModel', [C.c_void_p], None),
            ('mj_deleteSpec', [C.c_void_p], None),
        ]:
            fn = getattr(self.lib, name)
            fn.argtypes, fn.restype = args, result

    def compile(self, source, output=None):
        error = C.create_string_buffer(4096)
        spec = self.lib.mj_parseXML(os.fsencode(source), None, error, len(error))
        if not spec:
            raise RuntimeError(error.value.decode())
        model = None
        try:
            model = self.lib.mj_compile(spec, None)
            if not model:
                raise RuntimeError(self.lib.mjs_getError(spec).decode())
            if output is not None:
                if self.lib.mj_saveXML(spec, os.fsencode(output), error, len(error)) != 0:
                    raise RuntimeError(error.value.decode())
        finally:
            if model:
                self.lib.mj_deleteModel(model)
            self.lib.mj_deleteSpec(spec)


def fix_mesh_references(urdf, model):
    """Restore source mesh scales when the importer reuses a differently mirrored asset."""
    assets = model.find('asset')
    meshes = {m.get('name'): m for m in assets.findall('mesh')}
    geoms = {g.get('name'): g for g in model.iter('geom')}
    # Visuals as well as collisions: a bimanual description mirrors one arm from the other's
    # meshes, so a reused asset puts the wrong handedness on the arm the operator is driving.
    for collision in [*urdf.findall('link/collision'), *urdf.findall('link/visual')]:
        source = collision.find('geometry/mesh')
        if source is None or collision.get('name') not in geoms:
            continue
        geom = geoms[collision.get('name')]
        if geom.get('mesh') not in meshes:
            continue
        current = meshes[geom.get('mesh')]
        scale = source.get('scale', '1 1 1')
        expected = tuple(map(float, scale.split()))
        filename = str(Path(source.get('filename')).resolve())

        def matches(mesh):
            return (str(Path(mesh.get('file')).resolve()) == filename
                    and tuple(map(float, mesh.get('scale', '1 1 1').split())) == expected)

        if matches(current):
            continue
        replacement = next((m for m in meshes.values() if matches(m)), None)
        if replacement is None:
            replacement = copy.deepcopy(current)
            name = collision.get('name') + '_mesh'
            if name in meshes:
                raise RuntimeError(f'Duplicate mesh name: {name}')
            replacement.set('name', name)
            replacement.set('file', filename)
            replacement.set('scale', scale)
            assets.append(replacement)
            meshes[name] = replacement
        geom.set('mesh', replacement.get('name'))
        print('Corrected mesh reference:', collision.get('name'))


def prepare(args):
    folder = args.output
    description = args.description
    xacro = description / 'assets/robot/openarm_v1.0/urdf/openarm_v10.urdf.xacro'
    if not xacro.is_file():
        raise FileNotFoundError(f'OpenArm V1 description not found: {xacro}')
    mj = Mujoco(args.library)
    folder.mkdir(parents=True, exist_ok=True)
    source = folder / 'openarm_v1.urdf'
    run('ros2', 'run', 'xacro', 'xacro', xacro, 'bimanual:=true',
        'ros2_control:=false', '-o', source)

    urdf = ET.parse(source).getroot()
    for mesh in urdf.iter('mesh'):
        name = mesh.get('filename', '')
        prefix = 'package://openarm_description/'
        if not name.startswith(prefix):
            raise ValueError(f'Unexpected mesh URI: {name}')
        path = description / name[len(prefix):]
        if not path.is_file():
            raise FileNotFoundError(path)
        mesh.set('filename', str(path))
    extension = ET.SubElement(urdf, 'mujoco')
    # Visuals carry the materials and textures; discarding them drew the arm as collision hulls.
    ET.SubElement(extension, 'compiler', strippath='false', fusestatic='false',
                  discardvisual='false', balanceinertia='true')
    prepared = folder / 'openarm_v1_mujoco.urdf'
    write_xml(urdf, prepared)
    converted = folder / 'openarm_v1.xml'
    mj.compile(prepared, converted)
    model = ET.parse(converted).getroot()
    fix_mesh_references(urdf, model)
    for side in ('left', 'right'):
        if model.find(f".//body[@name='openarm_{side}_hand_tcp']") is None:
            raise RuntimeError(f'Missing {side} tool frame after conversion')
    write_xml(model, converted)
    mj.compile(converted)

    # A diagnostic motion model, deliberately not a contact/grasping simulation.
    option = model.find('option')
    if option is None:
        option = ET.SubElement(model, 'option')
    option.set('gravity', '0 0 0')
    option.set('integrator', 'implicitfast')
    flag = option.find('flag')
    if flag is None:
        flag = ET.SubElement(option, 'flag')
    flag.set('contact', 'disable')
    write_xml(model, folder / 'openarm_v1_motion_test.xml')

    actuators = ET.SubElement(model, 'actuator')
    joints = model.findall('./worldbody//joint')
    if len(joints) != 18:
        raise RuntimeError(f'Expected 14 arm and 4 finger joints; got {len(joints)}')
    for joint in joints:
        finger = joint.get('type') == 'slide'
        joint.set('damping', '2' if finger else '1')
        ET.SubElement(actuators, 'position', name=joint.get('name') + '_position',
                      joint=joint.get('name'), kp='100' if finger else '80',
                      kv='5' if finger else '8', ctrllimited='true',
                      ctrlrange=joint.get('range'))
    # URDF mimic joints are not retained by this conversion. Restore finger coupling.
    equality = ET.SubElement(model, 'equality')
    for joint in urdf.findall('joint'):
        mimic = joint.find('mimic')
        if mimic is not None:
            ET.SubElement(equality, 'joint', name=joint.get('name') + '_mimic',
                          joint1=joint.get('name'), joint2=mimic.get('joint'),
                          polycoef=f"{mimic.get('offset', '0')} {mimic.get('multiplier', '1')} 0 0 0")
    controlled = folder / 'openarm_v1_controlled.xml'
    # Down matches the user's RViz pose; bent remains available for IK experiments.
    home = []
    for joint in joints:
        name = joint.get('name')
        value = 0.0
        if args.home == 'bent' and name.endswith('_joint4'):
            value = 0.6
        elif args.home == 'bent' and name == 'openarm_right_joint2':
            value = 0.3
        elif args.home == 'bent' and name == 'openarm_left_joint2':
            value = -0.3
        elif joint.get('type') == 'slide':
            value = 0.02
        home.append(value)
    ET.SubElement(ET.SubElement(model, 'keyframe'), 'key', name='home',
                  qpos=' '.join(map(str, home)), ctrl=' '.join(map(str, home)))
    write_xml(model, controlled)
    mj.compile(controlled)

    configure(args)
    print(f'Validated model with {len(joints)} position actuators: {controlled}')
    if not args.skip_export:
        export(args)


def configure(args):
    """Refresh controller settings without reimporting or exporting robot geometry."""
    import yaml
    folder = args.output
    if not (folder / 'openarm_v1_controlled.xml').is_file():
        raise FileNotFoundError('Run prepare once before configure')
    config = yaml.safe_load((REPO / 'config/vive_vr.yaml').read_text())
    params = config.setdefault('vive_scene', {}).setdefault('ros__parameters', {})
    params.update(gravity_z=0.0, enable_grab=False, timestep=0.002, reset_keyframe=0)
    params['openarm.enabled'] = False
    (folder / 'preview.yaml').write_text(yaml.safe_dump(config))
    params['openarm.enabled'] = True
    params['openarm.require_alignment'] = True
    params['openarm.translation_scale'] = 1.0
    params['openarm.max_translation_m'] = 1.0
    params['openarm.max_rotation_rad'] = 1.75
    for arm in ('right', 'left'):
        hand = config.setdefault('vive_teleop', {}).setdefault('ros__parameters', {}).setdefault(
            'teleop', {}).setdefault(arm, {})
        filename = 'controller_alignment.json' if arm == 'right' else 'controller_alignment_left.json'
        alignment = folder / filename
        if alignment.is_file():
            record = json.loads(alignment.read_text())
            hand['tool_from_controller_rpy'] = record['tool_from_controller_rpy']
            prefix = 'openarm.' if arm == 'right' else 'openarm.left.'
            params[prefix + 'controller_to_tool_xyzw'] = record['quaternion_xyzw']
            params[prefix + 'alignment_configured'] = True
            print('Retained controller alignment:', alignment)
    (folder / 'teleop.yaml').write_text(yaml.safe_dump(config))


def export(args):
    # Group 1 only: the URDF importer puts visuals there and collision hulls in group 0, and
    # exporting both would draw the hulls over the meshes.
    run('ros2', 'run', 'vive_vr_ros2', 'scene_export',
        args.output / 'openarm_v1_controlled.xml', '-o', args.output / 'vr_scene_controlled',
        '--groups', '1')


def launch(args):
    params = 'teleop.yaml' if args.teleop else 'preview.yaml'
    for filename in ('openarm_v1_controlled.xml', params,
                     'vr_scene_controlled/manifest.json', 'vr_scene_controlled/scene.glb'):
        if not (args.output / filename).is_file():
            raise FileNotFoundError(f'Missing {filename}; run prepare first')
    os.environ.setdefault('ROS_DOMAIN_ID', '42')
    print('ROS_DOMAIN_ID=' + os.environ['ROS_DOMAIN_ID'], flush=True)
    if args.teleop and not (args.output / 'controller_alignment.json').is_file():
        print('No controller/tool alignment saved. Keep grip released and run align before motion tests.', flush=True)
    host_ip = args.host_ip
    if not host_ip:
        # Route lookup only; UDP connect sends no packet.
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            try:
                probe.connect(('192.0.2.1', 1))
                host_ip = probe.getsockname()[0]
            except OSError as error:
                raise RuntimeError('Cannot determine LAN address; pass --host-ip') from error
    command = ['ros2', 'launch', str(REPO / 'launch/openarm_sim.launch.py'),
               f'model:={args.output / "openarm_v1_controlled.xml"}',
               f'scene_dir:={args.output / "vr_scene_controlled"}',
               f'params_file:={args.output / params}',
               f'enable_teleop:={str(args.teleop).lower()}', f'host_ip:={host_ip}']
    print('+ ' + shlex.join(command), flush=True)
    if args.align_on_launch:
        calibrate_before_launch(command, args)
    os.execvp(command[0], command)


def calibrate_before_launch(command, args):
    """Own and stop only our temporary launch; never kill another running stack."""
    print(f'Put on the headset. Keep {args.arm} grip released and hand DOWN. '
          f'Alignment starts in {args.delay:g} seconds; teleop restarts automatically.', flush=True)
    process = subprocess.Popen(command + ['calibration_only:=true'], start_new_session=True)
    try:
        align(args)
        if process.poll() is not None:
            raise RuntimeError('Calibration simulation exited; check launch output')
    finally:
        # ROS launch forwards SIGINT to its children. Wait until its servers release their ports.
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
    print('Alignment saved. Restarting teleop now; headset will reconnect. '
          'Release grip, then hold it to move.', flush=True)


def reset(args):
    os.environ.setdefault('ROS_DOMAIN_ID', '42')
    run('ros2', 'service', 'call', '/vive_scene/reset', 'std_srvs/srv/Trigger', '{}')


def align(args):
    os.environ.setdefault('ROS_DOMAIN_ID', '42')
    run(sys.executable, REPO / 'scripts/openarm_align.py', '--output', args.output,
        '--delay', args.delay, '--timeout', 60, '--arm', args.arm)


def inputs(args):
    os.environ.setdefault('ROS_DOMAIN_ID', '42')
    run(sys.executable, REPO / 'scripts/openarm_inputs.py')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('prepare', 'configure', 'export', 'launch', 'reset', 'align', 'inputs'))
    parser.add_argument('--output', type=Path,
                        default=Path.home() / 'vive_vr_ws/models/openarm_v1')
    parser.add_argument('--description', type=Path,
                        default=Path.home() / 'openarm_ws/src/openarm_description')
    parser.add_argument('--library', type=Path, default=Path.home()
                        / '.cache/mj_kdl_wrapper/mujoco-3.9.0/lib/libmujoco.so.3.9.0')
    parser.add_argument('--skip-export', action='store_true', help='prepare model files only')
    parser.add_argument('--host-ip', help='PC address reachable by the headset (launch only)')
    parser.add_argument('--teleop', action='store_true', help='launch both arms with independent simulation IK')
    parser.add_argument('--align', dest='align_on_launch', action='store_true',
                        help='capture alignment then restart teleop automatically (launch --teleop only)')
    parser.add_argument('--arm', choices=('right', 'left'), default='right',
                        help='controller to calibrate with align or launch --align')
    parser.add_argument('--delay', type=float, default=10,
                        help='seconds to put on headset before alignment (default: 10)')
    parser.add_argument('--home', choices=('down', 'bent'), default='down',
                        help='starting/reset pose generated by prepare (default: down)')
    args = parser.parse_args()
    if args.align_on_launch and (args.command != 'launch' or not args.teleop):
        parser.error('--align requires launch --teleop')
    if not 0 <= args.delay <= 300:
        parser.error('--delay must be between 0 and 300 seconds')
    args.output = args.output.expanduser().resolve()
    args.description = args.description.expanduser().resolve()
    args.library = args.library.expanduser().resolve()
    try:
        {'prepare': prepare, 'configure': configure, 'export': export, 'launch': launch, 'reset': reset,
         'align': align, 'inputs': inputs}[args.command](args)
    except KeyboardInterrupt:
        parser.exit(130, 'Stopped.\n')
    except (OSError, RuntimeError, ValueError, subprocess.CalledProcessError) as error:
        parser.exit(1, f'Error: {error}\n')


if __name__ == '__main__':
    main()
