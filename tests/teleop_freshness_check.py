"""Real ROS transport regression: arrival freshness and release after dropout."""
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Empty
from vive_vr_ros2.msg import TeleopStatus
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data


def main():
    rclpy.init()
    node = rclpy.create_node('freshness_check')
    poses = node.create_publisher(PoseStamped, '/freshness/right/pose', qos_profile_sensor_data)
    joy = node.create_publisher(Joy, '/freshness/right/joy', qos_profile_sensor_data)
    toggle="--toggle" in sys.argv
    gate="--ready" in sys.argv
    ready_state = "align_pose" if gate else None
    statuses = node.create_publisher(TeleopStatus, "/freshness/teleop/right/status", qos_profile_sensor_data)
    reset=node.create_publisher(Empty,"/freshness/teleop/reset",10)
    events = []
    node.create_subscription(Bool, '/freshness/teleop/right/clutch',
                             lambda msg: events.append(msg.data),
                             QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    process = subprocess.Popen([sys.argv[1], '--ros-args', '-p', 'out_ns:=/freshness',
                                '-p', 'teleop.right.hand:=right', '-p', f'teleop.toggle_clutch:={str(toggle).lower()}', '-p', f'teleop.right.clutch_button:={0 if toggle else 1}', '-p', f'teleop.require_ready:={str(gate).lower()}'])

    def pump(seconds, pressed=None, publish_pose=True):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if ready_state is not None:
                status = TeleopStatus()
                status.header.stamp = node.get_clock().now().to_msg()
                status.state = ready_state
                status.ready = ready_state == 'ready'
                statuses.publish(status)
            if publish_pose:
                pose = PoseStamped()
                pose.header.frame_id = 'world'
                pose.header.stamp = (node.get_clock().now() - rclpy.duration.Duration(seconds=5)).to_msg()
                pose.pose.orientation.w = 1.0
                poses.publish(pose)
            if pressed is not None:
                msg = Joy()
                msg.buttons = [int(pressed),0] if toggle else [0,int(pressed)]
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
        if gate:
            pump(.25, False)
            events.clear()
            pump(.2, True)
            assert not any(events), 'Misaligned grip latched on'
            ready_state = 'ready'
            pump(.2, True)
            assert not any(events), 'Held rejected grip auto-engaged when green'
            pump(.1, False); pump(.15, True)
            assert events[-1], 'Fresh green press did not engage'
            ready_state = 'engaged'
            pump(.1, False); pump(.1, True)
            assert not events[-1], 'Toggle-off required readiness'
            ready_state = 'ready'
            pump(.1, False); pump(.1, True)
            ready_state = 'release_grip'
            pump(.15, True)
            assert not events[-1], 'Downstream rejection left toggle latched'
            count = sum(events)
            ready_state = 'ready'
            pump(.15, True)
            assert sum(events) == count, 'Rejected grip resumed without release'
            pump(.1, False)
            ready_state = None
            pump(.35, False); pump(.1, True)
            assert sum(events) == count, 'Stale readiness allowed engagement'
            print('PASS: rejected grip stays off, fresh green edge required, off always allowed, rejection rollback, stale status blocked')
            return
        pump(0.15, False)
        events.clear()
        pump(0.15, False)
        assert len(events) >= 2 and not any(events), 'Already-released grip was not refreshed'
        events.clear()
        pump(0.6, True)
        assert True in events, f'Fresh grip did not engage: {events}'
        # Release heartbeats already in DDS may arrive after events.clear().
        events[:] = events[events.index(True):]
        assert events == [True], f'Old timestamps caused clutch drops: {events}'
        pump(0.5, True, False)
        assert events == [True, False], f'Tracking loss did not stop once: {events}'
        pump(0.4, True)
        assert events == [True, False], 'Tracking recovery re-engaged a held grip'
        pump(0.1, False)
        pump(0.15, True)
        assert events[-1] and sum(events) == 2, 'Release/re-press did not recover'
        if toggle:
            pump(0.15,False)
            assert events[-1], 'release stopped toggled engagement'
            pump(0.15,True)
            assert not events[-1], 'second press did not stop'
            pump(0.1,False); pump(0.1,True); pump(0.1,False)
            assert events[-1], 'new toggle did not engage'
            pump(0.4,None)
            assert not events[-1], 'button dropout kept toggle engaged'
            count=sum(events); pump(0.15,True)
            assert sum(events)==count, 'held button resumed after loss'
            pump(0.1,False); pump(0.1,True)
            assert events[-1]
            reset.publish(Empty()); pump(0.15,True)
            assert not events[-1], 'reset did not stop toggle'
            count=sum(events); pump(0.15,True)
            assert sum(events)==count
            pump(0.1,False); pump(0.1,True)
            assert events[-1], 'fresh press after reset failed'
        print('PASS: idle release refresh, old timestamps, true dropout, no automatic re-engagement, release recovery')
    finally:
        process.terminate()
        process.wait(timeout=5)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
