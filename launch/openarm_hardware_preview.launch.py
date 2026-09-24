# SPDX-License-Identifier: MIT
"""Physical-feedback VR bridge: preview by default, explicit live mode with arming."""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)
    share = get_package_share_directory('vive_vr_ros2')
    with open(arg('params_file')) as stream:
        config = yaml.safe_load(stream)
    scene = config.get('vive_scene', {}).get('ros__parameters', {})
    # Reuse measured controller pairing only, never the simulation's motion limits.
    params = {key: value for key, value in scene.items() if key in (
        'openarm.alignment_configured', 'openarm.controller_to_tool_xyzw',
        'openarm.left.alignment_configured', 'openarm.left.controller_to_tool_xyzw')}
    params.update({
        'hardware.enable_commands': arg('enable_commands') == 'true',
        'hardware.control_grippers': arg('control_grippers') == 'true',
        'model': arg('model'),
        'manifest': os.path.join(arg('scene_dir'), 'manifest.json'),
        'scene_url': f"http://{arg('host_ip')}:{arg('http_port')}/scene.glb",
    })
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            share, 'launch', 'tracking.launch.py')),
            launch_arguments={'params_file': arg('params_file'),
                              'rosbridge_port': arg('rosbridge_port'),
                              'discovery_port': arg('discovery_port')}.items()),
        ExecuteProcess(cmd=['python3', '-m', 'http.server', arg('http_port'),
                            '--directory', arg('scene_dir')], output='screen'),
        Node(package='vive_vr_ros2', executable='teleop_node', name='vive_teleop',
             parameters=[arg('params_file')], output='screen'),
        Node(package='vive_vr_ros2', executable='openarm_hardware_preview_node',
             parameters=[arg('hardware_params_file'), params], output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model'), DeclareLaunchArgument('scene_dir'),
        DeclareLaunchArgument('params_file'), DeclareLaunchArgument('host_ip'),
        DeclareLaunchArgument('http_port', default_value='8000'),
        DeclareLaunchArgument('rosbridge_port', default_value='9090'),
        DeclareLaunchArgument('discovery_port', default_value='9091'),
        DeclareLaunchArgument('hardware_params_file', default_value=os.path.join(
            get_package_share_directory('vive_vr_ros2'), 'config', 'openarm_real.yaml')),
        DeclareLaunchArgument('enable_commands', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('control_grippers', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=setup),
    ])
