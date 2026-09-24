#!/usr/bin/env python3
"""Verify short trajectory and producer-loss hold on an ALREADY RUNNING mock driver."""
import time
import rclpy
from controller_manager_msgs.srv import ListHardwareComponents
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data

rclpy.init(); node=rclpy.create_node('mock_controller_check')
client=node.create_client(ListHardwareComponents,'/controller_manager/list_hardware_components')
assert client.wait_for_service(timeout_sec=5)
f=client.call_async(ListHardwareComponents.Request())
rclpy.spin_until_future_complete(node,f,timeout_sec=5)
assert f.done() and len(f.result().component)==2
assert all(c.plugin_name=='mock_components/GenericSystem' for c in f.result().component), 'Refusing: not mock hardware'
received=[]
sub=node.create_subscription(JointState,'/joint_states',received.append,qos_profile_sensor_data)
pub=node.create_publisher(JointTrajectory,'/right_joint_trajectory_controller/joint_trajectory',1)
def pump(duration):
    end=time.monotonic()+duration
    while time.monotonic()<end: rclpy.spin_once(node,timeout_sec=0.01)
pump(0.4)
assert received
names=[f'openarm_right_joint{i}' for i in range(1,8)]
initial=dict(zip(received[-1].name,received[-1].position))
msg=JointTrajectory(); msg.joint_names=names
point=JointTrajectoryPoint(); point.positions=[initial[n] for n in names]; point.positions[0]+=0.01
point.time_from_start.nanosec=100000000; msg.points=[point]
pub.publish(msg); pump(0.8)
q=dict(zip(received[-1].name,received[-1].position))
assert abs(q[names[0]]-point.positions[0])<1e-4
stream_start=q[names[0]]
for i in range(30):
 point.positions[0]=stream_start+0.02*(i+1)/30
 point.time_from_start.nanosec=40000000
 pub.publish(msg); pump(0.02)
pump(0.15)
q=dict(zip(received[-1].name,received[-1].position))
assert abs(q[names[0]]-(stream_start+0.02))<0.002, 'streamed trajectory did not advance'
before=q[names[0]]; pump(0.4)
q=dict(zip(received[-1].name,received[-1].position))
assert abs(q[names[0]]-before)<1e-5
print('PASS: mock-only single and streamed trajectories execute and hold after command stream ends')
node.destroy_node(); rclpy.shutdown()
