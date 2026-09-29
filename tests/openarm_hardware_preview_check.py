#!/usr/bin/env python3
"""End-to-end feedback gates and preview isolation, using synthetic ROS input only."""
import math
import os
os.environ['ROS_DOMAIN_ID'] = '195'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
import subprocess
import sys
import time
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory

rclpy.init()
node = rclpy.create_node('hardware_preview_check')
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
feedback = node.create_publisher(JointState, '/joint_states', qos_profile_sensor_data)
clutch = node.create_publisher(Bool, '/vive_vr/teleop/right/clutch', qos)
delta = node.create_publisher(TransformStamped, '/vive_vr/teleop/right/delta', qos_profile_sensor_data)
received = []
sub = node.create_subscription(JointTrajectory, '/vive_vr/hardware_preview/right/joint_trajectory', received.append, 10)
model = str(Path(__file__).with_name('openarm_test_arm.xml'))
proc = subprocess.Popen([sys.argv[1], '--ros-args', '-p', f'model:={model}', '-p', 'openarm.require_alignment:=false'], stdout=subprocess.DEVNULL)
names = [f'openarm_{side}_joint{i}' for side in ('right', 'left') for i in range(1, 8)] + [f'openarm_{side}_finger_joint1' for side in ('right','left')]

def pump(seconds, mode='valid', motion=False):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if mode != 'absent':
            m = JointState()
            m.header.stamp = node.get_clock().now().to_msg()
            m.name = names.copy()
            m.position = [0.0] * 14 + [0.01, 0.01]
            if mode == 'nan': m.position[0] = math.nan
            if mode == 'partial': m.name.pop(); m.position.pop()
            if mode == 'duplicate': m.name[-1] = m.name[0]
            if mode == 'stale': m.header.stamp.sec -= 2
            feedback.publish(m)
        if motion:
            d = TransformStamped()
            d.header.frame_id = 'right_tool_ref'
            d.child_frame_id = 'right_tool_cmd'
            d.transform.rotation.w = 1.0
            delta.publish(d)
        rclpy.spin_once(node, timeout_sec=0.01)
        time.sleep(0.005)
        assert proc.poll() is None, 'preview process exited'


def engage():
    clutch.publish(Bool(data=False)); pump(0.08)
    clutch.publish(Bool(data=True)); pump(0.15, motion=True)

try:
    pump(1.5, mode='absent')
    clutch.publish(Bool(data=False)); pump(0.1, mode='absent')
    clutch.publish(Bool(data=True)); pump(0.1, mode='absent', motion=True)
    assert not received, 'preview without feedback'
    pump(0.3, motion=True)
    assert not received, 'resumed on held grip after feedback recovery'
    engage()
    assert received, 'valid feedback and re-grip did not produce targets'
    assert len(received[-1].joint_names) == 8
    publishers = node.get_publisher_names_and_types_by_node('openarm_hardware_preview', '/')
    assert not any('controller/joint_trajectory' in name for name, _ in publishers), publishers
    for fault in ('absent', 'partial', 'nan', 'duplicate', 'stale'):
        pump(0.3, mode=fault, motion=True)
        count = len(received)
        pump(0.15, mode=fault, motion=True)
        assert len(received) == count, f'targets continued after {fault}'
        pump(0.2, motion=True)
        assert len(received) == count, f'auto-resumed after {fault}'
        engage()
        assert len(received) > count, f'cannot recover after {fault}'
    print('PASS: preview isolation, feedback validation, timeout and re-grip recovery')
finally:
    proc.terminate()
    try: proc.wait(timeout=5)
    except subprocess.TimeoutExpired: proc.kill(); proc.wait()
    node.destroy_node()
    rclpy.shutdown()
