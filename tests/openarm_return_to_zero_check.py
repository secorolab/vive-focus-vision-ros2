#!/usr/bin/env python3
"""Synthetic return-to-zero integration; no CAN and no physical robot."""
import os
os.environ['ROS_DOMAIN_ID']='194'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
import tempfile
import xml.etree.ElementTree as ET
import subprocess
import sys
import time
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import SetBool
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.msg import ControllerState

rclpy.init()
node=rclpy.create_node('zero_test')
qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
feedback=node.create_publisher(JointState,'/joint_states',qos_profile_sensor_data)
health=[node.create_publisher(Bool,f'/vive_vr/hardware/{s}/healthy',qos) for s in ('right','left')]
beat=node.create_publisher(Empty,'/openarm_hardware_preview/zero_keepalive',1)
client=node.create_client(SetBool,'/openarm_hardware_preview/return_to_zero')
enable=node.create_client(SetBool,'/openarm_hardware_preview/enable')
names=[f'openarm_{s}_joint{i}' for s in ('right','left') for i in range(1,8)]
names += [f'openarm_{s}_finger_joint1' for s in ('right','left')]
positions=dict.fromkeys(names,0.0)
positions[names[0]]=0.02; positions[names[7]]=-0.03
positions[names[14]]=positions[names[15]]=0.01
commands=[[],[]]; grippers=[]; status=[]
follow=True; healthy=True; feedback_on=True; keepalive=True; active=True

def receive(side,msg):
    commands[side].append(msg)
    if follow:
        positions.update(zip(msg.joint_names,msg.points[0].positions))
subs=[node.create_subscription(JointTrajectory,f'/{side}_joint_trajectory_controller/joint_trajectory',
                               lambda msg,i=i:receive(i,msg),10) for i,side in enumerate(('right','left'))]
subs += [node.create_subscription(JointTrajectory,f'/{s}_gripper_controller/joint_trajectory',grippers.append,10)
         for s in ('right','left')]
subs += [node.create_subscription(String,'/openarm_hardware_preview/zero_status',lambda m:status.append(m.data),qos)]
def controllers(req,res):
    res.controller=[ControllerState(name=f'{s}_{kind}',state='active' if active else 'inactive')
                    for s in ('right','left') for kind in ('joint_trajectory_controller','gripper_controller')]
    return res
service=node.create_service(ListControllers,'/controller_manager/list_controllers',controllers)
model_dir=tempfile.TemporaryDirectory()
model=Path(model_dir.name)/'arm.xml'
root=ET.parse(Path(__file__).with_name('openarm_test_arm.xml'))
for side in ('right','left'):
    root.find(f'.//joint[@name="openarm_{side}_joint4"]').set('range','0 2.4')
root.write(model)
positions['openarm_left_joint4']=-0.01049
positions['openarm_right_joint4']=-0.009
proc=subprocess.Popen([sys.argv[1],'--ros-args','-p',f'model:={model}',
                       '-p','hardware.enable_commands:=true','-p','hardware.control_grippers:=true'],stdout=subprocess.DEVNULL)
def pump(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        if feedback_on:
            msg=JointState(); msg.header.stamp=node.get_clock().now().to_msg()
            msg.name=names; msg.position=[positions[n] for n in names]; feedback.publish(msg)
        for pub in health: pub.publish(Bool(data=healthy))
        if keepalive: beat.publish(Empty())
        rclpy.spin_once(node,timeout_sec=0.002); time.sleep(0.003)
        assert proc.poll() is None

def call(which,value):
    f=which.call_async(SetBool.Request(data=value)); deadline=time.monotonic()+2
    while not f.done() and time.monotonic()<deadline: pump(0.01)
    assert f.done()
    return f.result()
try:
    healthy=False; pump(1.5)
    assert not call(client,True).success
    assert not any(commands)
    healthy=True; positions['openarm_left_joint4']=-0.03; pump(0.5)
    assert not call(client,True).success, 'accepted excessive elbow offset'
    positions['openarm_left_joint4']=-0.01049; pump(0.2)
    assert call(client,True).success
    assert not call(enable,True).success, 'VR enabled during return'
    assert not call(client,True).success, 'second return replaced active one'
    pump(3)
    assert status[-1]=='complete', status
    assert all(abs(positions[n])<0.01 for n in names[:14])
    assert not grippers, 'return moved grippers despite preserving them'
    for side in commands:
        assert side
        for a,b in zip(side,side[1:]):
            for before,after in zip(a.points[0].positions,b.points[0].positions):
                assert abs(after)<=abs(before)+1e-9, 'target reversed or overshot zero'
                assert abs(after-before)<=0.001001, 'exceeded 0.05 rad/s per 20ms'
    # Reproduce a static 0.64-degree residual: target reaches zero, measured
    # posture cannot reach the old 0.57-degree threshold with these gains.
    follow=False
    positions[names[0]]=0.0112; positions['openarm_left_joint4']=-0.01049
    pump(0.3); assert call(client,True).success
    pump(3); assert status[-1]=='complete', status
    assert commands[0][-1].points[0].positions[0]==0.0
    positions[names[0]]=0.025
    pump(0.3); assert call(client,True).success
    pump(3); assert status[-1]=='running', 'reported completion beyond one degree'
    assert call(client,False).success
    for failure in ('cancel','keepalive','health','feedback','controllers','following'):
        positions[names[0]]=0.15; positions[names[7]]=-0.15
        healthy=True; feedback_on=True; keepalive=True; active=True; follow=True
        pump(0.8); assert call(client,True).success
        pump(0.15)
        if failure=='cancel': assert call(client,False).success
        if failure=='keepalive': keepalive=False
        if failure=='health': healthy=False
        if failure=='feedback': feedback_on=False
        if failure=='controllers': active=False
        if failure=='following':
            follow=False; positions[names[0]]=0.40
        pump(1)
        assert status[-1].startswith('stopped:'), (failure,status)
        counts=[len(c) for c in commands]; pump(0.2)
        assert counts==[len(c) for c in commands], f'kept commanding after {failure}'
        healthy=True; feedback_on=True; keepalive=True; active=True; follow=True
        pump(0.8)
        assert counts==[len(c) for c in commands], 'automatically resumed'
    assert not grippers
    print('PASS: zero completion, bounded smooth path, VR exclusion, gripper preservation, cancel, lease and feedback/controller/following faults')
finally:
    proc.terminate()
    try: proc.wait(timeout=5)
    except subprocess.TimeoutExpired: proc.kill(); proc.wait()
    node.destroy_node(); rclpy.shutdown(); model_dir.cleanup()
