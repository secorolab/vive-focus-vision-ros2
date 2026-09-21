# SPDX-License-Identifier: MIT
"""OpenArm-specific simulation, leaving the generic scene launch unchanged."""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model'),
        DeclareLaunchArgument('scene_dir'),
        DeclareLaunchArgument('params_file'),
        DeclareLaunchArgument('host_ip', description='PC LAN address reachable by the headset'),
        DeclareLaunchArgument('http_port', default_value='8000'),
        DeclareLaunchArgument('enable_teleop', default_value='false'),
        DeclareLaunchArgument('calibration_only', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    share = get_package_share_directory('vive_vr_ros2')
    # Merge first: a node-specific YAML block otherwise overrides launch's wildcard
    # parameter dictionary, including the template's empty manifest value.
    with open(arg('params_file')) as stream:
        config = yaml.safe_load(stream) or {}
    scene_params = dict(config.get('/**', {}).get('ros__parameters', {}))
    scene_params.update(config.get('vive_scene', {}).get('ros__parameters', {}))
    scene_params.update({
        'model': arg('model'),
        'manifest': os.path.join(arg('scene_dir'), 'manifest.json'),
        'scene_url': f"http://{arg('host_ip')}:{arg('http_port')}/scene.glb",
    })
    if arg('calibration_only').lower() == 'true':
        # Stream controller/TCP poses for calibration, but never engage robot motion.
        scene_params['openarm.require_alignment'] = True
        scene_params['openarm.alignment_configured'] = False
        scene_params['openarm.left.alignment_configured'] = False
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'tracking.launch.py')),
            launch_arguments={'params_file': arg('params_file')}.items()),
        ExecuteProcess(cmd=['python3', '-m', 'http.server', arg('http_port'),
                            '--directory', arg('scene_dir')], output='screen'),
        Node(package='vive_vr_ros2', executable='openarm_sim_node', name='vive_scene',
             output='screen', parameters=[scene_params]),
        Node(package='vive_vr_ros2', executable='teleop_node', name='vive_teleop',
             parameters=[arg('params_file')], output='screen',
             condition=IfCondition(LaunchConfiguration('enable_teleop'))),
    ]
