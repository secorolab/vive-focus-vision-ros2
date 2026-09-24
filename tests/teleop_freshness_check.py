"""Real ROS transport regression: arrival freshness and release after dropout."""
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data


def main():
    rclpy.init()
    node = rclpy.create_node('freshness_check')
    poses = node.create_publisher(PoseStamped, '/freshness/right/pose', qos_profile_sensor_data)
    joy = node.create_publisher(Joy, '/freshness/right/joy', qos_profile_sensor_data)
    events = []
    node.create_subscription(Bool, '/freshness/teleop/right/clutch',
                             lambda msg: events.append(msg.data),
                             QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    process = subprocess.Popen([sys.argv[1], '--ros-args', '-p', 'out_ns:=/freshness',
                                '-p', 'teleop.right.hand:=right'])

    def pump(seconds, pressed=None, publish_pose=True):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if publish_pose:
                pose = PoseStamped()
                pose.header.frame_id = 'world'
                pose.header.stamp = (node.get_clock().now() - rclpy.duration.Duration(seconds=5)).to_msg()
                pose.pose.orientation.w = 1.0
                poses.publish(pose)
            if pressed is not None:
                msg = Joy()
                msg.buttons = [0, int(pressed)]
                msg.axes = [0.0, 0.0, 0.0]
                joy.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(0.01)

    try:
        deadline = time.monotonic() + 8
        while poses.get_subscription_count() == 0 or joy.get_subscription_count() == 0:
            if time.monotonic() > deadline:
                raise AssertionError('DDS discovery failed')
            pump(0.05, False)
        pump(0.15, False)
        events.clear()
        pump(0.15, False)
        assert len(events) >= 2 and not any(events), 'Already-released grip was not refreshed'
        events.clear()
        pump(0.6, True)
        assert events == [True], f'Old timestamps caused clutch drops: {events}'
        pump(0.5, True, False)
        assert events == [True, False], f'Tracking loss did not stop once: {events}'
        pump(0.4, True)
        assert events == [True, False], 'Tracking recovery re-engaged a held grip'
        pump(0.1, False)
        pump(0.15, True)
        assert events[-1] and sum(events) == 2, 'Release/re-press did not recover'
        print('PASS: idle release refresh, old timestamps, true dropout, no automatic re-engagement, release recovery')
    finally:
        process.terminate()
        process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
