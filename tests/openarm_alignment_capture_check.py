#!/usr/bin/env python3
"""Exercise alignment capture with synthetic ROS poses on the caller's test domain."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from rcl_interfaces.srv import SetParametersAtomically
from std_srvs.srv import SetBool
import yaml


def main():
    arm = sys.argv[1] if len(sys.argv) > 1 else 'right'
    mode = sys.argv[2] if len(sys.argv) > 2 else 'sim'
    prefix = 'hardware' if mode == 'real' else 'sim'
    filename = 'controller_alignment.json' if arm == 'right' else 'controller_alignment_left.json'
    rclpy.init()
    node=rclpy.create_node('alignment_capture_test')
    sensor=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    c=node.create_publisher(PoseStamped,f'/vive_vr/{arm}/pose',sensor)
    t=node.create_publisher(PoseStamped,f'/vive_vr/{prefix}/{arm}/ee_pose',sensor)
    grip=node.create_publisher(Bool,f'/vive_vr/teleop/{arm}/clutch',latched)
    updates = []
    def update(request, response):
        updates.append({p.name: p.value for p in request.parameters})
        response.result.successful = True
        return response
    target = '/openarm_hardware_preview' if mode == 'real' else '/vive_scene'
    node.create_service(SetParametersAtomically, target+'/set_parameters_atomically', update)
    node.create_service(SetParametersAtomically, '/vive_teleop/set_parameters_atomically', update)
    def disable(request, response):
        assert not request.data, 'Calibration attempted to arm the robot'
        response.success = True
        return response
    node.create_service(SetBool, '/openarm_hardware_preview/enable', disable)
    process=None
    try:
        with tempfile.TemporaryDirectory(prefix='alignment-test-') as directory:
            folder=Path(directory)
            (folder/'teleop.yaml').write_text(yaml.safe_dump(
                {'vive_teleop':{'ros__parameters':{'teleop':{arm:{}}}}}))
            script=Path(__file__).resolve().parents[1]/'scripts/openarm_align.py'
            process=subprocess.Popen([sys.executable,str(script),'--output',str(folder),
                                      '--delay','0','--timeout','6','--arm',arm,'--mode',mode],stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT,text=True)
            start=time.monotonic()
            while process.poll() is None and time.monotonic()-start<8:
                elapsed=time.monotonic()-start
                pressed=Bool();pressed.data=elapsed<0.8;grip.publish(pressed)
                for pub,quat in ((c,(0.,0.,0.,1.)),(t,(1.,0.,0.,0.))):
                    msg=PoseStamped();msg.header.frame_id='world'
                    msg.header.stamp=node.get_clock().now().to_msg()
                    msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w=quat
                    pub.publish(msg)
                if elapsed<0.8:
                    assert not (folder/filename).exists(), 'captured with clutch held'
                rclpy.spin_once(node,timeout_sec=0.02)
            if process.poll() is None:raise RuntimeError('alignment capture timed out')
            output=process.communicate()[0]
            assert process.returncode==0,output
            record=json.loads((folder/filename).read_text())
            assert record['samples']>=15
            assert abs(abs(record['tool_from_controller_rpy'][0])-180)<1e-6,record
            assert abs(record['tool_from_controller_rpy'][1])<1e-6,record
            assert abs(record['tool_from_controller_rpy'][2])<1e-6,record
            stored=yaml.safe_load((folder/'teleop.yaml').read_text())
            assert stored['vive_teleop']['ros__parameters']['teleop'][arm]['tool_from_controller_rpy']==record['tool_from_controller_rpy']
            key = 'openarm.' if arm == 'right' else 'openarm.left.'
            assert len(updates) == 3, updates
            assert not updates[0][key+'alignment_configured'].bool_value
            assert list(updates[1][f'teleop.{arm}.tool_from_controller_rpy'].double_array_value) == record['tool_from_controller_rpy']
            assert list(updates[2][key+'controller_to_tool_xyzw'].double_array_value) == record['quaternion_xyzw']
            assert updates[2][key+'alignment_configured'].bool_value
            assert 'No restart needed' in output
            print('PASS: released capture, saved alignment, gated live update of both nodes')
    finally:
        if process is not None and process.poll() is None:
            process.terminate();process.wait(timeout=3)
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
