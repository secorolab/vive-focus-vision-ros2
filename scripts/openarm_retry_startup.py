#!/usr/bin/env python3
"""Inspect or retry a blocked startup. Never homes or changes calibration."""
import argparse
import os
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='retry startup checks and enable at the measured pose; support the arms and keep the workspace clear')
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = '84'
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from controller_manager_msgs.srv import ListHardwareComponents, SetHardwareComponentState
    from std_srvs.srv import SetBool
    from std_msgs.msg import String
    rclpy.init()
    node = rclpy.create_node('openarm_retry_startup')
    diagnostics = {}
    subscriptions = []
    for side in ('right', 'left'):
        subscriptions.append(node.create_subscription(String, f'/vive_vr/hardware/{side}/diagnostic',
            lambda msg, side=side: diagnostics.update({side: msg.data}),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))

    def call(kind, name, request):
        client = node.create_client(kind, name)
        try:
            if not client.wait_for_service(timeout_sec=3):
                raise RuntimeError(f'{name} unavailable. Start the updated robot launch first.')
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=10)
            if not future.done():
                raise RuntimeError(f'{name} did not reply. State is uncertain; inspect the launch before retrying.')
            return future.result()
        finally:
            node.destroy_client(client)

    def show():
        end = time.monotonic() + 0.3
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
        for side in ('right', 'left'):
            print(f'{side.capitalize()}: {diagnostics.get(side, "No diagnostic received; inspect the robot launch.")}', flush=True)

    try:
        result = call(ListHardwareComponents, '/controller_manager/list_hardware_components', ListHardwareComponents.Request())
        components = {c.name: c for c in result.component}
        names = [f'openarm_{s}_hardware_interface' for s in ('right', 'left')]
        for name in names:
            if name not in components: raise RuntimeError(f'{name} not loaded. Check the launch.')
            print(f'{name}: {components[name].state.label}', flush=True)
        show()
        if not args.execute:
            print('Read-only check. After fixing the reported cause, add --execute to retry startup.')
            return 0
        if any(components[n].state.id not in (1, 2, 3) for n in names):
            raise RuntimeError('A driver is in an error/finalized state. Fix the fault and restart the launch.')
        if any(components[n].state.id == 3 for n in names) and any(components[n].state.id != 3 for n in names):
            raise RuntimeError('One arm is already active. Retrying the other can stall its feedback loop. Support the arms, stop the launch, and restart after fixing the reported cause.')
        response = call(SetBool, '/openarm_hardware_preview/enable', SetBool.Request(data=False))
        if not response.success: raise RuntimeError('Could not confirm VR disarmed: ' + response.message)
        print('VR disarmed. Checking both arms before activation. No homing or calibration changes.', flush=True)
        # Validate both unconfigured arms before enabling either one.
        for name in names:
            if components[name].state.id != 1: continue
            request = SetHardwareComponentState.Request(); request.name = name; request.target_state.id = 2
            response = call(SetHardwareComponentState, '/controller_manager/set_hardware_component_state', request)
            if not response.ok: raise RuntimeError('Startup still blocked. See the joint-specific explanation below.')
        for name in names:
            if components[name].state.id == 3: continue
            request = SetHardwareComponentState.Request(); request.name = name; request.target_state.id = 3
            response = call(SetHardwareComponentState, '/controller_manager/set_hardware_component_state', request)
            if not response.ok: raise RuntimeError('Activation blocked. Some motors may already hold position; inspect the explanation below.')
        controllers = ['joint_state_broadcaster', 'left_joint_trajectory_controller', 'right_joint_trajectory_controller',
                       'left_gripper_controller', 'right_gripper_controller']
        subprocess.run(['ros2', 'run', 'controller_manager', 'spawner', *controllers,
                        '--controller-manager', '/controller_manager'], check=True, timeout=60)
        print('Startup retry completed. VR remains disarmed. Enable it separately when ready.', flush=True)
        return 0
    except (RuntimeError, subprocess.SubprocessError) as exc:
        print(f'NOT READY: {exc}', flush=True)
        show()
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
