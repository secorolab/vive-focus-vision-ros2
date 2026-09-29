# SPDX-License-Identifier: MIT
"""VR simulation entry point. Never loads controllers or the CAN hardware driver."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as LC, PathJoinSubstitution


def generate_launch_description():
    share = get_package_share_directory('vive_vr_ros2')
    model_dir = LC('model_dir')
    return LaunchDescription([
        DeclareLaunchArgument('host_ip'),
        DeclareLaunchArgument('model_dir', default_value=os.path.expanduser('~/vive_vr_ws/models/openarm_v1')),
        DeclareLaunchArgument('sim_params_file', default_value=PathJoinSubstitution([model_dir,'teleop.yaml'])),
        DeclareLaunchArgument('http_port', default_value='8000'),
        DeclareLaunchArgument('rosbridge_port', default_value='9090'),
        DeclareLaunchArgument('discovery_port', default_value='9091'),
        # Fixed domains prevent a simulation publisher from reaching the real stack.
        SetEnvironmentVariable('ROS_DOMAIN_ID', '83'),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share,'launch','openarm_sim.launch.py')),
            launch_arguments={
                'model': PathJoinSubstitution([model_dir,'openarm_v1_controlled.xml']),
                'scene_dir': PathJoinSubstitution([model_dir,'vr_scene_controlled']),
                'params_file': LC('sim_params_file'),
                'host_ip': LC('host_ip'),
                'enable_teleop': 'true',
                **{key: LC(key) for key in ('http_port','rosbridge_port','discovery_port')},
            }.items()),
    ])
