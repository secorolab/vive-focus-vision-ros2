# SPDX-License-Identifier: MIT
"""Physical robot entry point. Uses measured feedback; starts VR disarmed."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC, PathJoinSubstitution


def generate_launch_description():
    share = get_package_share_directory('vive_vr_ros2')
    return LaunchDescription([
        DeclareLaunchArgument('host_ip'),
        DeclareLaunchArgument('model_dir', default_value=os.path.expanduser('~/vive_vr_ws/models/openarm_v1')),
        DeclareLaunchArgument('input_params_file', default_value=PathJoinSubstitution([LC('model_dir'),'teleop.yaml'])),
        DeclareLaunchArgument('hardware_params_file', default_value=os.path.join(share,'config','openarm_real.yaml')),
        DeclareLaunchArgument('control_grippers', default_value='false', choices=['true','false']),
        DeclareLaunchArgument('http_port', default_value='8000'),
        DeclareLaunchArgument('rosbridge_port', default_value='9090'),
        DeclareLaunchArgument('discovery_port', default_value='9091'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '84'),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share,'launch','openarm_live.launch.py')),
            launch_arguments={
                'use_fake_hardware': 'false',
                **{key: LC(key) for key in ('host_ip','model_dir','input_params_file',
                    'hardware_params_file','control_grippers','http_port','rosbridge_port','discovery_port')},
            }.items()),
    ])
