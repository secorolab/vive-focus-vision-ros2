#!/usr/bin/env python3
"""Return the physical arms to existing joint zero; never changes calibration."""
import argparse
import math
import os
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--execute', action='store_true',
                      help='move both arms along a direct joint-space path; ensure that path is clear')
    mode.add_argument('--cancel', action='store_true', help='stop a return and leave VR disarmed')
    mode.add_argument('--status', action='store_true', help='show measured arm angles without moving')
    args = parser.parse_args()
    # Match openarm_real.launch.py even when the calling shell uses the sim domain.
    os.environ['ROS_DOMAIN_ID'] = '84'
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import Empty, String
    from std_srvs.srv import SetBool
    from sensor_msgs.msg import JointState
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('openarm_return_to_zero_client')
    client = node.create_client(SetBool, '/openarm_hardware_preview/return_to_zero')
    heartbeat = node.create_publisher(Empty, '/openarm_hardware_preview/zero_keepalive', 1)
    status = [None, 0]
    def receive(msg):
        status[0] = msg.data
        status[1] += 1
    sub = node.create_subscription(String, '/openarm_hardware_preview/zero_status', receive,
                                  QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    names = [f'openarm_{side}_joint{i}' for side in ('right','left') for i in range(1,8)]
    measured = [{}, 0.0]
    def joints(msg):
        age = (node.get_clock().now().nanoseconds - (msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))/1e9
        if len(msg.name)!=len(msg.position) or len(set(msg.name))!=len(msg.name) or not -0.1<=age<=0.15:
            measured[1]=0; return
        values = dict(zip(msg.name,msg.position))
        if all(n in values and math.isfinite(values[n]) for n in names):
            measured[:] = [values, time.monotonic()]
        else: measured[1]=0
    joint_sub = node.create_subscription(JointState, '/joint_states', joints, qos_profile_sensor_data)
    def show_positions(full=False):
        if time.monotonic()-measured[1]>0.15:
            print('Joint feedback unavailable/stale; zero position is NOT confirmed.', flush=True)
            return False
        q=measured[0]
        if full:
            print('Measured joint angles in degrees (J1 through J7):', flush=True)
            for side in ('right','left'):
                print(f"  {side:5s}: " + '  '.join(f'{math.degrees(q[f"openarm_{side}_joint{i}"]):+.2f}' for i in range(1,8)), flush=True)
        worst=max(abs(q[n]) for n in names)
        print(f"Farthest joint from zero: {math.degrees(worst):.2f} deg; "
              f"left elbow: {math.degrees(q['openarm_left_joint4']):+.2f} deg; "
              f"right elbow: {math.degrees(q['openarm_right_joint4']):+.2f} deg", flush=True)
        if full:
            print('Within zero tolerance (1 degree).' if worst<=math.radians(1) else 'Not at zero yet.', flush=True)
        return True
    accepted = False
    uncertain = False
    def call(value, timeout=3):
        future = client.call_async(SetBool.Request(data=value))
        deadline = time.monotonic()+timeout
        while not future.done() and time.monotonic()<deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        if not future.done():
            raise RuntimeError('Return service did not reply; no completion confirmed')
        return future.result()
    try:
        if args.status:
            deadline=time.monotonic()+5
            while time.monotonic()<deadline and time.monotonic()-measured[1]>0.15:
                rclpy.spin_once(node, timeout_sec=0.02)
            return 0 if show_positions(full=True) else 1
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError('Start the updated openarm_real launch first. No return service found on domain 84.')
        # Drain an old transient status before starting this request.
        end = time.monotonic()+0.2
        while time.monotonic()<end:
            rclpy.spin_once(node, timeout_sec=0.02)
        baseline = status[1]
        if args.execute:
            show_positions(full=True)
            print('Requesting return of BOTH arms to joint zero, max 0.05 rad/s. Grippers unchanged.', flush=True)
            print('Direct path: keep it clear. Ctrl+C stops the return; it does not release motor torque.', flush=True)
        uncertain = args.execute
        response = call(args.execute)
        uncertain = False
        if not response.success:
            raise RuntimeError(response.message)
        print(response.message, flush=True)
        if args.cancel:
            return 0
        accepted = True
        deadline = time.monotonic()+180
        next_heartbeat = 0
        next_progress = 0
        while time.monotonic()<deadline:
            now = time.monotonic()
            if now>=next_heartbeat:
                heartbeat.publish(Empty()); next_heartbeat=now+0.1
            rclpy.spin_once(node, timeout_sec=0.02)
            if now>=next_progress:
                show_positions(); next_progress=now+1
            if status[1]>baseline:
                if status[0]=='complete':
                    accepted=False
                    show_positions(full=True)
                    print('Both arms reached the zero target within 1 degree. VR remains disarmed.', flush=True)
                    return 0
                if status[0].startswith('stopped:'):
                    raise RuntimeError(status[0])
        raise RuntimeError('Return timed out')
    except KeyboardInterrupt:
        print('\nStopping return; VR remains disarmed.', file=sys.stderr)
        return 130
    except Exception as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 1
    finally:
        if accepted or uncertain:
            try:
                print(call(False).message, file=sys.stderr)
            except Exception:
                print('Could not confirm cancellation; the bridge stops on missing keepalive within 0.5 s.', file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
