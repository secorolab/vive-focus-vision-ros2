#!/usr/bin/env python3
"""Synthetic live bridge tests on an isolated ROS domain; never opens CAN."""
import os
os.environ['ROS_DOMAIN_ID'] = '196'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
import subprocess
import sys
import time
import math
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState, Joy
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
                         '-p', 'hardware.enable_commands:=true', '-p', 'hardware.control_grippers:=true',
                         '-p', 'openarm.alignment_configured:=true',
                         '-p', 'openarm.left.alignment_configured:=true'], stdout=subprocess.DEVNULL)
names = [f'openarm_{side}_joint{i}' for side in ('right','left') for i in range(1,8)] + [f'openarm_{side}_finger_joint1' for side in ('right','left')]

left_gripper_commands=[]
subs.append(node.create_subscription(JointTrajectory, '/left_gripper_controller/joint_trajectory', left_gripper_commands.append, 10))
buttons=[0,0,0]
stick=0.0
locked=True
joy_on=True
pose_on=True
joy=node.create_publisher(Joy, '/vive_vr/right/joy', qos_profile_sensor_data)
neutral_joy={side:node.create_publisher(Joy,f'/vive_vr/{side}/joy',qos_profile_sensor_data) for side in ('right','left')}

def pump(seconds, healthy=True, feedback_on=True, delta_on=True, health_on=True, delta_x=0.01, delta_angle=0.0):
    end = time.monotonic()+seconds
    while time.monotonic()<end:
        if feedback_on:
            msg=JointState(); msg.header.stamp=node.get_clock().now().to_msg()
            msg.name=names; msg.position=[0.0]*14+[gripper_commands[-1].points[0].positions[0] if gripper_commands else 0.01, left_gripper_commands[-1].points[0].positions[0] if left_gripper_commands else 0.01]; feedback.publish(msg)
        if health_on:
            for pub in health: pub.publish(Bool(data=healthy))
        if joy_on:
            j=Joy(); j.header.stamp=node.get_clock().now().to_msg(); j.buttons=[0,0,0,0,0,0,int(locked)]; j.axes=[0.0,stick,0.0,0.0]; joy.publish(j)
            neutral=Joy(); neutral.header.stamp=j.header.stamp
            neutral.buttons=[0,0,0,0,0,0,int(locked)]; neutral.axes=[0.0]*4
            neutral_joy['left' if joy.topic_name.endswith('/right/joy') else 'right'].publish(neutral)
        for pub in poses if pose_on else []:
            p=PoseStamped(); p.header.stamp=node.get_clock().now().to_msg(); p.header.frame_id='world'; p.pose.orientation.w=1.0; pub.publish(p)
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

try:
    stick=-1.0; pump(1.5); arm(); pump(0.15)
    assert not gripper_commands, 'deflected stick started on enable'
    stick=0.0; pump(0.1); stick=-1.0; pump(0.3)
    assert len(gripper_commands)>3
    values=[m.points[0].positions[0] for m in gripper_commands]
    assert values[-1]<values[0]<=0.01
    assert all(0<=a-b<=0.000300001 for a,b in zip(values,values[1:]))
    assert not commands and not left_commands and not left_gripper_commands
    stick=0.0; pump(0.1); count=len(gripper_commands); pump(0.15)
    assert len(gripper_commands)==count, 'neutral did not stop'
    stick=1.0; pump(0.3); before=gripper_commands[-1].points[0].positions[0]; pump(0.15)
    assert gripper_commands[-1].points[0].positions[0]>before
    joy_on=False; pump(0.4); count=len(gripper_commands); pump(0.1)
    assert len(gripper_commands)==count
    joy_on=True; pump(0.15)
    assert len(gripper_commands)==count, 'stick resumed without neutral'
    stick=0.0; pump(0.1); stick=-1.0; pump(0.2)
    assert len(gripper_commands)>count
    pose_on=False; pump(0.4); count=len(gripper_commands); pose_on=True; pump(0.2)
    assert len(gripper_commands)==count
    stick=0.0; pump(0.1); stick=1.0; pump(0.2)
    locked=False; pump(0.2); count=len(gripper_commands); locked=True; pump(0.2)
    assert len(gripper_commands)==count, 'unlock did not disarm'
    stick=0.0; arm(); pump(0.1)
    joy=node.create_publisher(Joy, '/vive_vr/left/joy', qos_profile_sensor_data)
    pump(0.4); stick=-1.0; pump(0.2)
    assert left_gripper_commands[-1].points[0].positions[0]<0.01
    stick=0.0; pump(0.1); before=left_gripper_commands[-1].points[0].positions[0]
    stick=1.0; pump(0.3)
    assert left_gripper_commands[-1].points[0].positions[0]>before
    print('Stick direction, neutral stop, lock, freshness and isolation checks passed')

finally:
    proc.terminate()
    try: proc.wait(timeout=5)
    except subprocess.TimeoutExpired: proc.kill(); proc.wait()
    node.destroy_node(); rclpy.shutdown()
