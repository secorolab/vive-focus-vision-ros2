"""Exercise launch configuration without starting nodes or opening CAN interfaces."""
import importlib.util
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET
import unittest
from unittest.mock import patch
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.utilities import perform_substitutions, normalize_to_list_of_substitutions

ROOT = Path(__file__).resolve().parents[1]


def load(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'launch' / (name + '.launch.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Modes(unittest.TestCase):
    def entry(self, name):
        module = load(name)
        context = LaunchContext()
        context.launch_configurations['host_ip'] = '127.0.0.1'
        with patch.object(module, 'get_package_share_directory', return_value=str(ROOT)):
            actions = module.generate_launch_description().entities
        includes = []
        for action in actions:
            if isinstance(action, (DeclareLaunchArgument, SetEnvironmentVariable)):
                action.execute(context)
            elif isinstance(action, IncludeLaunchDescription):
                includes.append(action)
        self.assertEqual(len(includes), 1)
        # Resolve/read the included description, but never execute its actions.
        includes[0].launch_description_source.get_launch_description(context)
        args = {k: perform_substitutions(context, normalize_to_list_of_substitutions(v))
                for k, v in includes[0].launch_arguments}
        return context, includes[0], args

    def test_virtual_is_sim_only_and_isolated(self):
        context, include, args = self.entry('openarm_virtual')
        self.assertEqual(context.environment['ROS_DOMAIN_ID'], '83')
        self.assertTrue(include.launch_description_source.location.endswith('openarm_sim.launch.py'))
        self.assertEqual(args['enable_teleop'], 'true')
        self.assertNotIn('use_fake_hardware', args)

    def test_real_is_physical_and_isolated(self):
        context, include, args = self.entry('openarm_real')
        self.assertEqual(context.environment['ROS_DOMAIN_ID'], '84')
        self.assertTrue(include.launch_description_source.location.endswith('openarm_live.launch.py'))
        self.assertEqual(args['use_fake_hardware'], 'false')
        self.assertTrue(args['hardware_params_file'].endswith('openarm_real.yaml'))

    def test_startup_allowances_keep_original_driver_limits(self):
        module=load('openarm_hardware')
        for fake in ('false','true'):
            root=ET.Element('robot')
            for side in ('right','left'):
                control=ET.SubElement(root,'ros2_control')
                hardware=ET.SubElement(control,'hardware')
                ET.SubElement(hardware,'plugin').text='original_plugin'
                for i in range(1,9):
                    name=f'openarm_{side}_joint{i}' if i<8 else f'openarm_{side}_finger_joint1'
                    joint=ET.SubElement(root,'joint',name=name)
                    ET.SubElement(joint,'limit',lower='0' if i in (4,8) else '-2',upper='0.044' if i==8 else '2.4')
                    c=ET.SubElement(control,'joint',name=name)
                    ET.SubElement(c,'command_interface',name='position')
            context=LaunchContext()
            context.launch_configurations.update(use_fake_hardware=fake,right_can_interface='can0',left_can_interface='can1')
            with patch.object(module.xacro,'process_file') as xacro, \
                    patch.object(module.resource,'getrlimit',return_value=(50,50)), \
                    patch.object(module,'get_package_share_directory',return_value=str(ROOT)), \
                    patch.object(module,'Node') as node:
                xacro.return_value.toxml.return_value=ET.tostring(root,encoding='unicode')
                module.setup(context)
            generated=ET.fromstring(node.call_args_list[0].kwargs['parameters'][0]['robot_description'])
            for side in ('right','left'):
                limit=generated.find(f'joint[@name="openarm_{side}_joint4"]/limit')
                self.assertEqual(float(limit.get('lower')), -0.02 if fake=='false' else 0)
                grip=generated.find(f'joint[@name="openarm_{side}_finger_joint1"]/limit')
                self.assertEqual(float(grip.get('lower')), -0.0002 if fake=='false' else 0)
                self.assertEqual(float(grip.get('upper')),0.044)
            for control in generated.findall('ros2_control'):
                self.assertEqual(control.find('hardware/param[@name="lower3"]').text,'0')
                self.assertEqual(control.find('hardware/param[@name="lower7"]').text,'0')
                self.assertEqual(control.find('hardware/param[@name="upper7"]').text,'0.044')
                self.assertEqual(control.find('hardware/plugin').text,
                                 'vive_vr_ros2/GuardedOpenArm' if fake=='false' else 'original_plugin')

    def test_sim_tuning_does_not_leak_to_hardware(self):
        module = load('openarm_hardware_preview')
        config = {'vive_scene': {'ros__parameters': {
            'openarm.translation_scale': 99.0, 'openarm.joint_speed_rad_s': 99.0,
            'openarm.enabled': True, 'openarm.alignment_configured': True,
            'openarm.controller_to_tool_xyzw': [0, 0, 0, 1]}}}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'teleop.yaml'
            path.write_text(yaml.safe_dump(config))
            context = LaunchContext()
            context.launch_configurations.update({
                'params_file': str(path), 'enable_commands': 'false', 'control_grippers': 'false',
                'model': 'model.xml', 'scene_dir': directory, 'host_ip': '127.0.0.1',
                'http_port': '8001', 'rosbridge_port': '9092', 'discovery_port': '0',
                'hardware_params_file': str(ROOT/'config/openarm_real.yaml')})
            with patch.object(module, 'get_package_share_directory', return_value=str(ROOT)), \
                    patch.object(module, 'Node') as node:
                module.setup(context)
            params = node.call_args_list[-1].kwargs['parameters']
            self.assertTrue(params[0].endswith('openarm_real.yaml'))
            self.assertNotIn('openarm.translation_scale', params[1])
            self.assertNotIn('openarm.joint_speed_rad_s', params[1])
            self.assertTrue(params[1]['openarm.alignment_configured'])
            self.assertFalse(params[1]['hardware.enable_commands'])


if __name__ == '__main__':
    unittest.main()
