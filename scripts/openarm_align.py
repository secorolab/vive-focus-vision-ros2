#!/usr/bin/env python3
"""Measure a controller/tool orientation pairing in the shared VR scene.

Run with grip released, robot at home, controller held to represent the tool.
Writes configuration only. Never commands the robot. Restart teleop afterwards.
"""
import argparse
import json
import math
from pathlib import Path
import time


def multiply(a, b):
    """Quaternion product, xyzw order."""
    x,y,z,w = a
    X,Y,Z,W = b
    return (w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z)


def normalized(q):
    length = math.sqrt(sum(v*v for v in q))
    if not math.isfinite(length) or not 0.9 <= length <= 1.1:
        raise ValueError('Invalid orientation in pose stream')
    return tuple(v/length for v in q)


def pairing(controller, tool):
    # Matches TeleopNode: Delta_tool = R_tc^-1 Delta_controller R_tc.
    c = normalized(controller)
    return normalized(multiply((-c[0], -c[1], -c[2], c[3]), normalized(tool)))


def degrees_rpy(q):
    x,y,z,w = q
    sin_pitch = 2*(w*y-z*x)
    if abs(sin_pitch) > 1-1e-10:
        # At gimbal lock choose yaw=0 and retain the equivalent roll.
        return [math.degrees(math.atan2(2*(w*x-y*z), 1-2*(x*x+z*z))),
                math.copysign(90.,sin_pitch), 0.]
    return [math.degrees(math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))),
            math.degrees(math.asin(max(-1.,min(1.,sin_pitch)))),
            math.degrees(math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))]


def main():
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Bool
    import yaml

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arm', choices=('right', 'left'), default='right')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=20)
    parser.add_argument('--delay', type=float, default=10, help='seconds to get into the alignment pose')
    args = parser.parse_args()
    config_path = args.output / 'teleop.yaml'
    if not config_path.is_file():
        parser.error('Run openarm_sim.py prepare first')
    config = yaml.safe_load(config_path.read_text())
    namespace = config.get('/**', {}).get('ros__parameters', {}).get('out_ns', '/vive_vr')
    rclpy.init()
    node = rclpy.create_node('openarm_align')
    sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    poses = {}
    grip = [None]
    samples = []
    previous_stamp = None
    def pose(name, msg):
        poses[name] = (msg, time.monotonic())
    node.create_subscription(PoseStamped, namespace+f'/{args.arm}/pose',
                             lambda m: pose('controller', m), sensor)
    node.create_subscription(PoseStamped, namespace+f'/sim/{args.arm}/ee_pose',
                             lambda m: pose('tool', m), sensor)
    node.create_subscription(Bool, namespace+f'/teleop/{args.arm}/clutch',
                             lambda m: grip.__setitem__(0, m.data), latched)
    print(f'Keep {args.arm} grip RELEASED. Hold the controller still in the orientation that represents')
    print('the hanging gripper: the controller direction you choose as the tool tip points DOWN.')
    print(f'You have {args.delay:g} seconds to get into position.')
    print('Capturing one second of steady poses; the robot will not be commanded.', flush=True)
    ready = time.monotonic() + args.delay
    deadline = ready + args.timeout
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            if time.monotonic() < ready:
                continue
            if grip[0] is not False or len(poses) != 2:
                samples.clear()
                continue
            c, ctime = poses['controller']
            t, ttime = poses['tool']
            stamp = (c.header.stamp.sec, c.header.stamp.nanosec)
            if stamp == previous_stamp:
                continue
            previous_stamp = stamp
            now = time.monotonic()
            ros_now = node.get_clock().now().nanoseconds / 1e9
            if (now-ctime > 0.25 or now-ttime > 0.25 or
                c.header.frame_id != 'world' or t.header.frame_id != 'world' or
                any(not -0.05 <= ros_now-(m.header.stamp.sec+m.header.stamp.nanosec/1e9) <= 0.25
                    for m in (c,t))):
                samples.clear()
                continue
            cq = c.pose.orientation; tq = t.pose.orientation
            q = pairing((cq.x,cq.y,cq.z,cq.w), (tq.x,tq.y,tq.z,tq.w))
            if samples:
                dot = sum(a*b for a,b in zip(samples[0][1],q))
                if 2*math.acos(min(1,abs(dot))) > math.radians(2):
                    samples.clear()
                elif dot < 0:
                    q = tuple(-v for v in q)
            samples.append((now,q))
            if len(samples) >= 15 and now-samples[0][0] >= 1:
                q = normalized(tuple(sum(s[1][i] for s in samples)/len(samples) for i in range(4)))
                rpy = degrees_rpy(q)
                config['vive_teleop']['ros__parameters']['teleop'][args.arm]['tool_from_controller_rpy'] = rpy
                record = {'tool_from_controller_rpy': rpy, 'quaternion_xyzw': list(q),
                          'method': 'matched controller and TCP orientations in VR world',
                          'samples': len(samples)}
                (args.output / ('controller_alignment.json' if args.arm == 'right' else 'controller_alignment_left.json')).write_text(json.dumps(record, indent=2)+'\n')
                params = config.setdefault('vive_scene', {}).setdefault('ros__parameters', {})
                prefix = 'openarm.' if args.arm == 'right' else 'openarm.left.'
                params[prefix + 'controller_to_tool_xyzw'] = list(q)
                params[prefix + 'alignment_configured'] = True
                params['openarm.require_alignment'] = True
                config_path.write_text(yaml.safe_dump(config))
                print('Saved tool_from_controller_rpy (degrees):', [round(v,3) for v in rpy])
                print('Restart launch --teleop to apply. Keep the same controller grip for later use.')
                return 0
        print('No steady fresh pose pair with released grip. Check tracking, ROS domain and robot launch.')
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
