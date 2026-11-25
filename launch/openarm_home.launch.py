# SPDX-License-Identifier: MIT
"""Local homing services: no headset, IP, web server or VR command input required."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    # Avoid starting another CAN owner alongside an existing robot controller.
    for process in Path('/proc').glob('[0-9]*'):
        try:
            executable = (process/'cmdline').read_bytes().split(b'\0')[0].decode()
        except (OSError, UnicodeError):
            continue
        if Path(executable).name == 'ros2_control_node':
            raise RuntimeError('A robot controller is already running. Stop the existing robot launch before starting local homing.')
    share = get_package_share_directory('vive_vr_ros2')
    model = LaunchConfiguration('model_dir').perform(context)
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'openarm_hardware.launch.py')),
                                 launch_arguments={'use_fake_hardware': 'false', 'local_elbow_recovery': 'true'}.items()),
        Node(package='vive_vr_ros2', executable='openarm_hardware_preview_node', output='screen',
             parameters=[os.path.join(share, 'config', 'openarm_real.yaml'), {
                 'model': os.path.join(model, 'openarm_v1_controlled.xml'),
                 'manifest': os.path.join(model, 'vr_scene_controlled', 'manifest.json'),
                 'hardware.enable_commands': True,
                 'hardware.control_grippers': True,
                 'hardware.homing_only': True,
             }]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model_dir', default_value=os.path.expanduser('~/vive_vr_ws/models/openarm_v1')),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '84'),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        OpaqueFunction(function=setup),
    ])
