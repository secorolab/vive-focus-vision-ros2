#!/usr/bin/env python3
"""Synthetic live bridge tests on an isolated ROS domain; never opens CAN."""
import os
os.environ['ROS_DOMAIN_ID'] = '193'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
import subprocess
import sys
import time
import math
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, PoseStamped
from std_msgs.msg import Bool, Float32
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.msg import ControllerState

rclpy.init()
node = rclpy.create_node('live_bridge_check')
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
feedback = node.create_publisher(JointState, '/joint_states', qos_profile_sensor_data)
clutch = node.create_publisher(Bool, '/vive_vr/teleop/right/clutch', qos)
trigger = node.create_publisher(Float32, '/vive_vr/teleop/right/gripper', 10)
delta = node.create_publisher(TransformStamped, '/vive_vr/teleop/right/delta', qos_profile_sensor_data)
poses = [node.create_publisher(PoseStamped, f'/vive_vr/{side}/pose', qos_profile_sensor_data) for side in ('right','left')]
health = [node.create_publisher(Bool, f'/vive_vr/hardware/{side}/healthy', qos) for side in ('right','left')]
commands, left_commands, gripper_commands = [], [], []
subs = [node.create_subscription(JointTrajectory, '/right_joint_trajectory_controller/joint_trajectory', commands.append, 10),
        node.create_subscription(JointTrajectory, '/left_joint_trajectory_controller/joint_trajectory', left_commands.append, 10),
        node.create_subscription(JointTrajectory, '/right_gripper_controller/joint_trajectory', gripper_commands.append, 10)]
controller_active = True

def controller_list(request, response):
    response.controller = [ControllerState(name=f'{side}_{kind}', state='active' if controller_active else 'inactive')
                           for side in ('right','left') for kind in ('joint_trajectory_controller','gripper_controller')]
    return response
service = node.create_service(ListControllers, '/controller_manager/list_controllers', controller_list)
client = node.create_client(SetBool, '/openarm_hardware_preview/enable')
model = str(Path(__file__).with_name('openarm_test_arm.xml'))
proc = subprocess.Popen([sys.argv[1], '--ros-args', '-p', f'model:={model}',
                         '-p', 'hardware.enable_commands:=true',
                         '-p', 'openarm.alignment_configured:=true',
                         '-p', 'openarm.left.alignment_configured:=true'], stdout=subprocess.DEVNULL)
names = [f'openarm_{side}_joint{i}' for side in ('right','left') for i in range(1,8)] + [f'openarm_{side}_finger_joint1' for side in ('right','left')]

def pump(seconds, healthy=True, feedback_on=True, delta_on=True, health_on=True, delta_x=0.01, delta_angle=0.0):
    end = time.monotonic()+seconds
    while time.monotonic()<end:
        if feedback_on:
            msg=JointState(); msg.header.stamp=node.get_clock().now().to_msg()
            msg.name=names; msg.position=[0.0]*14+[0.01,0.01]; feedback.publish(msg)
        if health_on:
            for pub in health: pub.publish(Bool(data=healthy))
        for pub in poses:
            p=PoseStamped(); p.header.frame_id='world'; p.pose.orientation.w=1.0; pub.publish(p)
        if delta_on:
            d=TransformStamped(); d.header.frame_id='right_tool_ref'; d.child_frame_id='right_tool_cmd'
            d.transform.rotation.w=math.cos(delta_angle/2); d.transform.rotation.z=math.sin(delta_angle/2)
            d.transform.translation.x=delta_x; delta.publish(d)
        rclpy.spin_once(node, timeout_sec=0.005); time.sleep(0.005)
        assert proc.poll() is None, 'bridge exited'

def arm():
    f=client.call_async(SetBool.Request(data=True))
    deadline=time.monotonic()+2
    while not f.done() and time.monotonic()<deadline: pump(0.02)
    assert f.done() and f.result().success, f.result()

def grip():
    clutch.publish(Bool(data=False)); pump(0.08)
    clutch.publish(Bool(data=True)); pump(0.18)

try:
    pump(1.5, healthy=False)
    assert client.service_is_ready()
    f=client.call_async(SetBool.Request(data=True)); pump(0.1, healthy=False)
    assert f.done() and not f.result().success, 'armed without hardware health'
    assert 'right driver reports a fault' in f.result().message
    assert 'left driver reports a fault' in f.result().message
    pump(0.5); grip()
    assert not commands, 'commanded before explicit enable'
    arm(); pump(0.1)
    assert not commands, 'enable with held grip caused movement'
    grip()
    assert commands, 'healthy armed re-grip did not command'
    assert all(len(m.joint_names)==7 and len(m.points[0].positions)==7 for m in commands)
    assert not left_commands, 'right controller commanded left arm'
    assert not gripper_commands, 'grippers moved despite disabled gripper control'
    trigger.publish(Float32(data=0.0)); pump(0.1)
    trigger.publish(Float32(data=1.0)); pump(2.2)
    count=len(commands); pump(0.1)
    assert len(commands)>count, 'disabled gripper target disarmed the arm'
    count=len(commands); pump(0.1, delta_x=2.0, delta_angle=2.0)
    assert len(commands)>count, 'large hand offset dropped the grip instead of capping the target'
    count=len(commands); pump(0.1)
    assert len(commands)>count, 'capped target failed to resume within the same grip'
    for a,b in zip(commands,commands[1:]):
        assert max(abs(x-y) for x,y in zip(a.points[0].positions,b.points[0].positions)) <= 0.006001
    for fault in ('health_false','health_timeout','feedback_timeout','controller_inactive','tracking_timeout'):
        controller_active = fault != 'controller_inactive'
        kw={'healthy':fault!='health_false','health_on':fault!='health_timeout',
            'feedback_on':fault!='feedback_timeout','delta_on':fault!='tracking_timeout'}
        pump(1.0, **kw)
        count=len(commands); pump(0.2, **kw)
        assert len(commands)==count, f'commands continued during {fault}'
        controller_active=True; pump(0.5)
        assert len(commands)==count, f'auto resumed after {fault}'
        arm(); grip()
        assert len(commands)>count, f'could not resume after {fault}'
    print('PASS: explicit arm, grip gating, split seven-joint output, speed bound, arm isolation, gripper gate, health/feedback/controller/tracking stops')
finally:
    proc.terminate()
    try: proc.wait(timeout=5)
    except subprocess.TimeoutExpired: proc.kill(); proc.wait()
    node.destroy_node(); rclpy.shutdown()
