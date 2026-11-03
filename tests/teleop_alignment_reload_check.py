"""Verify live calibration changes outgoing tool-frame deltas, without hardware."""
import subprocess
import sys
import time

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import SetParametersAtomically
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import Joy


def main():
    rclpy.init()
    node = rclpy.create_node('alignment_reload_test')
    poses = node.create_publisher(PoseStamped, '/reload/right/pose', qos_profile_sensor_data)
    buttons = node.create_publisher(Joy, '/reload/right/joy', qos_profile_sensor_data)
    received = []
    node.create_subscription(TransformStamped, '/reload/teleop/right/delta',
                             lambda msg: received.append(msg), qos_profile_sensor_data)
    process = subprocess.Popen([sys.argv[1], '--ros-args', '-p', 'out_ns:=/reload',
                                '-p', 'teleop.right.hand:=right'])
    client = node.create_client(SetParametersAtomically, '/vive_teleop/set_parameters_atomically')

    def pump(x, down, seconds=.1):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            pose = PoseStamped()
            pose.pose.orientation.w = 1.
            pose.pose.position.x = x
            poses.publish(pose)
            buttons.publish(Joy(axes=[0., 0., 0.], buttons=[0, int(down)]))
            rclpy.spin_once(node, timeout_sec=.01)

    def update(values):
        future = client.call_async(SetParametersAtomically.Request(parameters=[
            Parameter('teleop.right.tool_from_controller_rpy', value=values).to_parameter_msg()]))
        rclpy.spin_until_future_complete(node, future, timeout_sec=1.)
        assert future.done(), 'parameter service timed out'
        return future.result().result.successful

    try:
        assert client.wait_for_service(timeout_sec=8.)
        pump(0., False, .5)
        assert not update([0., float('nan'), 0.]), 'accepted nonfinite calibration'
        assert update([0., 0., 90.]), 'disengaged calibration rejected'
        pump(0., False)
        pump(0., True)
        received.clear()
        pump(.02, True)
        assert received, 'no tool deltas'
        delta = received[-1].transform.translation
        assert abs(delta.x) < 1e-6 and abs(delta.y + .02) < 1e-6, delta
        assert not update([0., 0., 0.]), 'accepted calibration while following'
        pump(0., False)
        assert update([0., 0., 0.])
        pump(0., False)
        pump(0., True)
        received.clear()
        pump(.02, True)
        delta = received[-1].transform.translation
        assert abs(delta.x - .02) < 1e-6 and abs(delta.y) < 1e-6, delta
        print('PASS: live mapping changes tool deltas; invalid/engaged updates rejected')
    finally:
        process.terminate()
        process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
