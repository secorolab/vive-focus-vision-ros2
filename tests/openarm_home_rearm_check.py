"""Automatic VR re-enable checks against a fake service; no hardware."""
import os
os.environ['ROS_DOMAIN_ID'] = '196'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
import sys
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from openarm_return_to_zero import enable_vr_after_return

rclpy.init()
node=rclpy.create_node('home_rearm_test')
mode='released'
requests=[]
def enable(req,res):
    requests.append(req.data)
    res.success=mode!='fault' or not req.data
    res.message='synthetic feedback fault' if not res.success else 'OK'
    return res
service=node.create_service(SetBool,'/openarm_hardware_preview/enable',enable)
joys={side:node.create_publisher(Joy,f'/vive_vr/{side}/joy',qos_profile_sensor_data) for side in ('right','left')}
poses={side:node.create_publisher(PoseStamped,f'/vive_vr/{side}/pose',qos_profile_sensor_data) for side in joys}
def publish():
    for side in joys:
        msg=Joy()
        msg.header.stamp=node.get_clock().now().to_msg()
        msg.buttons=[0]*6+[int(mode!='unlocked')]
        if mode=='held': msg.buttons[0]=1
        if mode=='stale': msg.header.stamp.sec-=2
        joys[side].publish(msg)
        pose=PoseStamped()
        pose.header.stamp=msg.header.stamp
        pose.pose.orientation.w=1.
        if mode!='no_tracking': poses[side].publish(pose)
timer=node.create_timer(.02,publish)
try:
    for mode in ('held','unlocked','no_tracking','stale','fault','released'):
        requests.clear()
        enabled,message=enable_vr_after_return(node,timeout=1.5)
        if mode=='released':
            assert enabled and requests==[True],(mode,requests,message)
        elif mode=='fault':
            assert not enabled and requests==[True,False],(mode,requests,message)
        else:
            assert not enabled and not requests,(mode,requests,message)
    print('PASS: re-enable needs fresh tracking, locked world, released buttons and healthy enable response')
finally:
    node.destroy_node()
    rclpy.shutdown()
