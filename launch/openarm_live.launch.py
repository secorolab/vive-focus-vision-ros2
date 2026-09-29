# SPDX-License-Identifier: MIT
"""Guarded OpenArm bring-up and VR bridge; explicit service arming is required."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    share = get_package_share_directory('vive_vr_ros2')
    model_dir = LaunchConfiguration('model_dir')
    return LaunchDescription([
        DeclareLaunchArgument('host_ip'),
        DeclareLaunchArgument('model_dir', default_value=os.path.expanduser('~/vive_vr_ws/models/openarm_v1')),
        DeclareLaunchArgument('use_fake_hardware', default_value='true', choices=['true','false']),
        DeclareLaunchArgument('control_grippers', default_value='false', choices=['true','false']),
        DeclareLaunchArgument('http_port', default_value='8000'),
        DeclareLaunchArgument('rosbridge_port', default_value='9090'),
        DeclareLaunchArgument('discovery_port', default_value='9091'),
        DeclareLaunchArgument('hardware_params_file', default_value=os.path.join(share,'config','openarm_real.yaml')),
        DeclareLaunchArgument('input_params_file', default_value=PathJoinSubstitution([model_dir,'teleop.yaml'])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share,'launch','openarm_hardware.launch.py')),
            launch_arguments={'use_fake_hardware': LaunchConfiguration('use_fake_hardware')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share,'launch','openarm_hardware_preview.launch.py')),
            launch_arguments={
                'model': PathJoinSubstitution([model_dir,'openarm_v1_controlled.xml']),
                'scene_dir': PathJoinSubstitution([model_dir,'vr_scene_controlled']),
                'params_file': LaunchConfiguration('input_params_file'),
                'hardware_params_file': LaunchConfiguration('hardware_params_file'),
                'http_port': LaunchConfiguration('http_port'),
                'rosbridge_port': LaunchConfiguration('rosbridge_port'),
                'discovery_port': LaunchConfiguration('discovery_port'),
                'host_ip': LaunchConfiguration('host_ip'),
                'enable_commands': 'true',
                'control_grippers': LaunchConfiguration('control_grippers'),
            }.items()),
    ])
