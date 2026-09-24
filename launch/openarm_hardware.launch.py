# SPDX-License-Identifier: MIT
"""Headless V1 bring-up. Fake hardware by default; guarded real activation holds measured positions."""
import os
import resource
import xacro
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)
    rt_limit = resource.getrlimit(resource.RLIMIT_RTPRIO)[0]
    if arg('use_fake_hardware') == 'false' and rt_limit != resource.RLIM_INFINITY and rt_limit < 50:
        raise RuntimeError('Real hardware requires realtime priority allowance 50. '
                           'In your terminal run: sudo prlimit --pid $$ --rtprio=50:50 '
                           'then launch again from that same terminal. See docs/openarm_hardware.md.')
    description = xacro.process_file(os.path.join(
        get_package_share_directory('openarm_description'),
        'assets/robot/openarm_v1.0/urdf/openarm_v10.urdf.xacro'), mappings={
            'arm_type': 'v1.0', 'bimanual': 'true', 'ros2_control': 'true',
            'use_fake_hardware': arg('use_fake_hardware'),
            'right_can_interface': arg('right_can_interface'),
            'left_can_interface': arg('left_can_interface'),
        }).toxml()
    root = ET.fromstring(description)
    limits = {j.attrib['name']: j.find('limit') for j in root.findall('joint')}
    for control in root.findall('ros2_control'):
        hardware = control.find('hardware')
        if arg('use_fake_hardware') == 'false':
            hardware.find('plugin').text = 'vive_vr_ros2/GuardedOpenArm'
        for i, joint in enumerate(control.findall('joint')):
            for interface in list(joint.findall('command_interface')):
                if interface.attrib['name'] != 'position':
                    joint.remove(interface)
            limit = limits[joint.attrib['name']]
            for bound in ('lower', 'upper'):
                ET.SubElement(hardware, 'param', name=f'{bound}{i}').text = limit.attrib[bound]
    # Only the ROS controller's envelope gets small startup/readback allowances.
    # GuardedOpenArm received the ORIGINAL limits above and enforces those at the
    # motor boundary, allowing an out-of-range starting target only to hold/return.
    if arg('use_fake_hardware') == 'false':
        for name in ('openarm_right_joint4', 'openarm_left_joint4'):
            limits[name].set('lower', str(float(limits[name].get('lower')) - 0.02))
        # The closed gripper can report slightly below zero (-0.136 mm observed).
        # Accept that measured hold without repeated clamping/logging. The driver
        # still owns the original 0..0.044 m range and its inward-only corridor.
        for name in ('openarm_right_finger_joint1', 'openarm_left_finger_joint1'):
            limits[name].set('lower', str(float(limits[name].get('lower')) - 0.0002))
    description = ET.tostring(root, encoding='unicode')
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}], output='screen'),
        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[{'robot_description': description}, os.path.join(
                 get_package_share_directory('vive_vr_ros2'), 'config',
                 'openarm_hardware_controllers.yaml')], output='screen'),
        Node(package='controller_manager', executable='spawner', arguments=[
            'joint_state_broadcaster', 'left_joint_trajectory_controller',
            'right_joint_trajectory_controller', 'left_gripper_controller',
            'right_gripper_controller', '--controller-manager', '/controller_manager'],
             output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_fake_hardware', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('right_can_interface', default_value='can0'),
        DeclareLaunchArgument('left_can_interface', default_value='can1'),
        OpaqueFunction(function=setup),
    ])
