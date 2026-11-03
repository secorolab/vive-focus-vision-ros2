#!/usr/bin/env python3
"""Return the physical arms to existing joint zero; never changes calibration."""
import argparse
import math
import os
import sys
import time


def enable_vr_after_return(node, timeout=10.):
    """Re-enable once after confirmed arrival; never resumes a previous grip."""
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Joy
    from geometry_msgs.msg import PoseStamped
    from std_srvs.srv import SetBool

    buttons, poses = {}, {}
    subscriptions = []
    client = node.create_client(SetBool, '/openarm_hardware_preview/enable')
    def fresh(stamp):
        age = (node.get_clock().now().nanoseconds-(stamp.sec*10**9+stamp.nanosec))/1e9
        return -0.1 <= age <= .25
    def joy(side, msg):
        valid = fresh(msg.header.stamp) and len(msg.buttons) >= 7
        buttons[side] = (time.monotonic(), valid and msg.buttons[6] == 1 and
                         all(value == 0 for value in msg.buttons[:6]))
    def pose(side, msg):
        p, q = msg.pose.position, msg.pose.orientation
        values = (p.x,p.y,p.z,q.x,q.y,q.z,q.w)
        valid = fresh(msg.header.stamp) and all(math.isfinite(v) for v in values)
        valid = valid and .81 <= q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w <= 1.21
        poses[side] = (time.monotonic(), valid)
    for side in ('right','left'):
        subscriptions.append(node.create_subscription(Joy, f'/vive_vr/{side}/joy',
            lambda msg, side=side: joy(side,msg), qos_profile_sensor_data))
        subscriptions.append(node.create_subscription(PoseStamped, f'/vive_vr/{side}/pose',
            lambda msg, side=side: pose(side,msg), qos_profile_sensor_data))
    attempted = False
    confirmed = False
    try:
        print('At zero. Release controller buttons; waiting to enable VR automatically.', flush=True)
        end, released_since = time.monotonic()+timeout, None
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
            now = time.monotonic()
            ready = all(side in stream and stream[side][1] and now-stream[side][0] <= .25
                        for stream in (buttons,poses) for side in ('left','right'))
            if not ready:
                released_since = None
                continue
            if released_since is None: released_since = now
            if now-released_since < .3 or not client.service_is_ready(): continue
            attempted = True
            future = client.call_async(SetBool.Request(data=True))
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.)
            if not future.done():
                return False, 'VR enable did not reply; disabling again. Check the robot panel.'
            response = future.result()
            confirmed = response.success
            return confirmed, ('VR arms and grippers enabled. Press side grip on green to follow.'
                               if confirmed else 'At zero, but VR could not enable: '+response.message)
        return False, 'At zero; VR stays disabled. Lock world, track both controllers and release buttons, then Enable VR.'
    except BaseException:
        confirmed = False
        raise
    finally:
        if attempted and not confirmed:
            future = client.call_async(SetBool.Request(data=False))
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.)
        for subscription in subscriptions: node.destroy_subscription(subscription)
        node.destroy_client(client)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--execute', action='store_true',
                      help='move both arms along a direct joint-space path; ensure that path is clear')
    mode.add_argument('--cancel', action='store_true', help='stop a return and leave VR disarmed')
    mode.add_argument('--status', action='store_true', help='show measured arm angles without moving')
    parser.add_argument('--close-grippers', action='store_true', help='also close both empty grippers to their existing zero target')
    parser.add_argument('--enable-vr-after', action='store_true',
                        help='after successful return only, enable VR once with released, tracked controllers')
    args = parser.parse_args()
    if args.enable_vr_after and not args.execute: parser.error('--enable-vr-after requires --execute')
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
    service_name='return_to_zero_and_close' if args.close_grippers else 'return_to_zero'
    client = node.create_client(SetBool, '/openarm_hardware_preview/'+service_name)
    heartbeat = node.create_publisher(Empty, '/openarm_hardware_preview/zero_keepalive', 1)
    status = [None, 0]
    def receive(msg):
        status[0] = msg.data
        status[1] += 1
    sub = node.create_subscription(String, '/openarm_hardware_preview/zero_status', receive,
                                  QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    names = [f'openarm_{side}_joint{i}' for side in ('right','left') for i in range(1,8)]
    finger_names=[f'openarm_{side}_finger_joint1' for side in ('right','left')]
    measured = [{}, 0.0]
    def joints(msg):
        age = (node.get_clock().now().nanoseconds - (msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))/1e9
        if len(msg.name)!=len(msg.position) or len(set(msg.name))!=len(msg.name) or not -0.1<=age<=0.15:
            measured[1]=0; return
        values = dict(zip(msg.name,msg.position))
        if all(n in values and math.isfinite(values[n]) for n in names+(finger_names if args.close_grippers else [])):
            measured[:] = [values, time.monotonic()]
        else: measured[1]=0
    joint_sub = node.create_subscription(JointState, '/joint_states', joints, qos_profile_sensor_data)
    def show_positions(full=False):
        if time.monotonic()-measured[1]>0.15:
            print('Cannot read the current arm positions. Zero is NOT confirmed. Check the robot launch for the first STARTUP BLOCKED or LATCHED FAULT message.', flush=True)
            return False
        q=measured[0]
        if full:
            print('Measured joint angles in degrees (J1 through J7):', flush=True)
            for side in ('right','left'):
                print(f"  {side:5s}: " + '  '.join(f'{math.degrees(q[f"openarm_{side}_joint{i}"]):+.2f}' for i in range(1,8)), flush=True)
        if args.close_grippers:
            print('Gripper positions (mm; closed target = 0): ' + ', '.join(
                f'{side}: {q[n]*1000:+.2f}' for side,n in zip(('right','left'),finger_names)), flush=True)
        worst_name=max(names, key=lambda n: abs(q[n]))
        worst=abs(q[worst_name])
        print(f"Farthest joint from zero: {math.degrees(worst):.2f} deg ({worst_name}); "
              f"left elbow: {math.degrees(q['openarm_left_joint4']):+.2f} deg; "
              f"right elbow: {math.degrees(q['openarm_right_joint4']):+.2f} deg", flush=True)
        if full:
            print('Within zero tolerance (1.5 degrees).' if worst<=math.radians(1.5) else 'Not at zero yet.', flush=True)
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
            raise RuntimeError('Robot return service is unavailable. Start openarm_real.launch.py and wait for both arm controllers to activate (ROS domain 84).')
        # Drain an old transient status before starting this request.
        end = time.monotonic()+0.2
        while time.monotonic()<end:
            rclpy.spin_once(node, timeout_sec=0.02)
        baseline = status[1]
        if args.execute:
            if not show_positions(full=True):
                raise RuntimeError('Return was not requested: live joint positions are required. Fix the startup/motor fault, then restart the robot launch. Enabling VR will not clear a motor fault.')
            print('Requesting return of BOTH arms to joint zero, max 0.10 rad/s. ' +
                  ('Closing both grippers; keep them empty and clear.' if args.close_grippers else 'Grippers unchanged.'), flush=True)
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
                    print('Both arms reached zero within 1.5 degrees. ' +
                          ('Grippers reached the closed target within 1 mm. ' if args.close_grippers else '') +
                          'VR remains disarmed.', flush=True)
                    if args.enable_vr_after:
                        enabled, message = enable_vr_after_return(node)
                        print(message, flush=True)
                        return 0 if enabled else 3
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
