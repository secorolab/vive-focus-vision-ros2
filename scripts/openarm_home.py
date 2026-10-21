#!/usr/bin/env python3
"""Start local hardware, return to stored zero, and keep holding until Ctrl+C."""
import argparse
import fcntl
import math
import os
from pathlib import Path
import resource
import signal
import subprocess
import sys
import tempfile
import time


def stop_owned(process):
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=12)
    except subprocess.TimeoutExpired:
        print(f'Process {process.pid} has not stopped. Inspect it before starting another controller.', flush=True)


def feedback_ready(now, measured_at, health):
    return 0 <= now-measured_at <= .15 and all(
        health.get(side, (False, 0))[0] and 0 <= now-health[side][1] <= .15
        for side in ('right', 'left'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', required=True, help='enable motors and move BOTH arms; keep the direct path clear')
    parser.add_argument('--close-grippers', action='store_true', help='also close both empty grippers')
    args = parser.parse_args()
    allowance = resource.getrlimit(resource.RLIMIT_RTPRIO)[0]
    if allowance != resource.RLIM_INFINITY and allowance < 50:
        parser.error('Run sudo prlimit --pid $$ --rtprio=50:50 in this terminal first.')
    for entry in Path('/proc').glob('[0-9]*'):
        try:
            executable = entry.joinpath('cmdline').read_bytes().split(b'\0')[0].decode()
        except (OSError, UnicodeError):
            continue
        if Path(executable).name == 'ros2_control_node':
            parser.error('A robot controller is already running. Stop its launch before using this command.')
    lock = (Path.home()/'.openarm_home.lock').open('a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        parser.error('Another local homing command is already running.')
    os.environ['ROS_DOMAIN_ID'] = '84'
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from std_msgs.msg import Bool, String
    from sensor_msgs.msg import JointState
    from controller_manager_msgs.srv import ListControllers
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('openarm_local_home_supervisor')
    health = {}; reasons = {}; measured_at = 0.0
    names = {f'openarm_{side}_joint{i}' for side in ('right','left') for i in range(1,8)}
    names |= {f'openarm_{side}_finger_joint1' for side in ('right','left')}
    def joints(msg):
        nonlocal measured_at
        age = (node.get_clock().now().nanoseconds - msg.header.stamp.sec*10**9-msg.header.stamp.nanosec)/1e9
        values = dict(zip(msg.name, msg.position))
        valid = (len(msg.name)==len(msg.position)==len(values) and -.1<=age<=.15
                 and names<=values.keys() and all(math.isfinite(values[n]) for n in names))
        measured_at = time.monotonic() if valid else 0.0
    subscriptions = [node.create_subscription(JointState, '/joint_states', joints, qos_profile_sensor_data)]
    for side in ('right','left'):
        subscriptions.append(node.create_subscription(Bool, f'/vive_vr/hardware/{side}/healthy',
            lambda msg, side=side: health.update({side:(msg.data,time.monotonic())}),
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
        subscriptions.append(node.create_subscription(String, f'/vive_vr/hardware/{side}/diagnostic',
            lambda msg, side=side: reasons.update({side:msg.data}),
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    def fresh():
        now=time.monotonic()
        return feedback_ready(now, measured_at, health)
    controllers = node.create_client(ListControllers, '/controller_manager/list_controllers')
    expected = {'joint_state_broadcaster','right_joint_trajectory_controller','left_joint_trajectory_controller',
                'right_gripper_controller','left_gripper_controller'}
    launch = returning = None
    log_path = Path(tempfile.mkdtemp(prefix='openarm-home-'))/'launch.log'
    log = log_path.open('w')
    result=1
    try:
        print(f'Starting local hardware. No headset or IP needed. Full log: {log_path}',flush=True)
        print('Both arms will return along a direct path. No table-collision checking. Ctrl+C ends holding and may release torque: support the arms first.',flush=True)
        launch = subprocess.Popen(['ros2','launch','vive_vr_ros2','openarm_home.launch.py'],
                                  stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        deadline=time.monotonic()+30; future=None; next_check=0; ready=False
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.02)
            if launch.poll() is not None: raise RuntimeError('Local hardware launch exited. Read the log below.')
            if any('blocked' in r.lower() or 'failed' in r.lower() or r.startswith('Stopped:') for r in reasons.values()):
                raise RuntimeError('Startup check failed; no return requested.')
            if future is not None and future.done():
                response=future.result(); future=None
                if response and expected<={c.name for c in response.controller if c.state=='active'} and fresh():
                    ready=True; break
            if future is None and time.monotonic()>=next_check and controllers.service_is_ready():
                future=controllers.call_async(ListControllers.Request()); next_check=time.monotonic()+.25
        if not ready: raise RuntimeError('Hardware did not become ready within 30 seconds; no return requested.')
        print('Fresh feedback and controllers confirmed. Returning to stored zero.',flush=True)
        command=[sys.executable,str(Path(__file__).with_name('openarm_return_to_zero.py')),'--execute']
        if args.close_grippers: command.append('--close-grippers')
        returning=subprocess.Popen(command,start_new_session=True)
        while returning.poll() is None:
            rclpy.spin_once(node,timeout_sec=.05)
            if launch.poll() is not None: raise RuntimeError('Hardware launch exited during return.')
        if returning.returncode: raise RuntimeError('Return did not complete. No automatic retry.')
        result=0
        print('NEAR ZERO CONFIRMED: all arm joints within 1.5 degrees. Keeping motors enabled. Support the arms before Ctrl+C; shutdown may release torque.',flush=True)
        while True:
            rclpy.spin_once(node,timeout_sec=.05)
            if launch.poll() is not None or not fresh():
                raise RuntimeError('Feedback/driver health lost. Holding and zero position are no longer confirmed.')
    except (RuntimeError, OSError) as exc:
        result=1
        stop_owned(returning)
        print(f'NOT READY: {exc}',flush=True)
        for side, reason in reasons.items(): print(f'{side.capitalize()}: {reason}',flush=True)
        print(f'Details: {log_path}\nNo further movement requested. Services remain running; torque state depends on the fault. Support the arms, then press Ctrl+C to stop.',flush=True)
        try:
            while launch is not None and launch.poll() is None:
                rclpy.spin_once(node,timeout_sec=.1)
        except KeyboardInterrupt:
            pass
    except KeyboardInterrupt:
        print('\nStopping return and local services. Shutdown may release torque.',flush=True)
    finally:
        stop_owned(returning)
        stop_owned(launch)
        log.close(); node.destroy_node(); rclpy.shutdown(); lock.close()
    return result


if __name__ == '__main__':
    raise SystemExit(main())
