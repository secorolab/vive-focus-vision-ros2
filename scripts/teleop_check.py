#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Vamsi Kalagaturu
# See LICENSE for details.
"""Drives the raw topics like the headset would and checks what vive_teleop publishes.

Poses stream continuously on a timer, as they do from the device; the dropout test is the one
place they deliberately stop.
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32

SENSOR = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
LATCHED = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

fails = []


def check(name, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}")
    if not ok:
        fails.append(name)


class Driver(Node):
    def __init__(self):
        super().__init__("teleop_check")
        self.pose_pub = self.create_publisher(PoseStamped, "/vive_vr/raw/right/pose", SENSOR)
        self.joy_pub = self.create_publisher(Joy, "/vive_vr/raw/right/joy", SENSOR)
        self.deltas = []
        self.clutch = []
        self.at = (1.0, 0.0, 1.0, 0.0)
        self.streaming = True
        self.grips = []
        self.create_subscription(TransformStamped, "/vive_vr/teleop/right/delta", self.on_delta, SENSOR)
        self.create_subscription(Bool, "/vive_vr/teleop/right/clutch", self.on_clutch, LATCHED)
        self.create_subscription(Float32, "/vive_vr/teleop/right/gripper", self.on_grip, SENSOR)
        self.create_timer(1.0 / 50.0, self.tick)

    def on_delta(self, msg):
        self.deltas.append(msg)

    def on_clutch(self, msg):
        self.clutch.append(msg.data)

    def on_grip(self, msg):
        self.grips.append(msg.data)

    def tick(self):
        if not self.streaming:
            return
        x, y, z, yaw = self.at
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
        half = math.radians(yaw) / 2.0
        msg.pose.orientation.z = math.sin(half)
        msg.pose.orientation.w = math.cos(half)
        self.pose_pub.publish(msg)

    def button(self, down, trigger=0.0):
        """Clutch is the grip button (index 1); the trigger axis drives the gripper."""
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.buttons = [0, 1 if down else 0, 0, 0, 0, 0]
        msg.axes = [0.0, 0.0, trigger, 0.0]
        self.joy_pub.publish(msg)

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.01)


def main():
    rclpy.init()
    d = Driver()
    d.spin(1.5)

    d.deltas.clear()
    d.button(True)
    d.spin(0.4)

    check("clutch closes on press", d.clutch and d.clutch[-1] is True, f"saw {d.clutch}")
    if d.deltas:
        t = d.deltas[0].transform.translation
        r = d.deltas[0].transform.rotation
        near_identity = (abs(t.x) < 1e-6 and abs(t.y) < 1e-6 and abs(t.z) < 1e-6
                         and abs(abs(r.w) - 1.0) < 1e-6)
        check("delta is identity at the press", near_identity,
              f"t=({t.x:.6f},{t.y:.6f},{t.z:.6f}) w={r.w:.6f}")
    else:
        check("delta is identity at the press", False, "no delta published")
        return 1

    d.at = (1.0, 0.0, 1.1, 0.0)
    d.spin(0.4)
    t = d.deltas[-1].transform.translation
    check("10 cm up gives dz = 0.100", abs(t.z - 0.1) < 1e-4 and abs(t.x) < 1e-4,
          f"dz={t.z:.6f} dx={t.x:.6f} dy={t.y:.6f}")

    d.at = (1.0, 0.0, 1.0, 90.0)
    d.spin(0.4)
    r = d.deltas[-1].transform.rotation
    expect = math.sin(math.radians(90) / 2)
    check("90 deg yaw appears as qz = sin(45)", abs(r.z - expect) < 1e-4,
          f"qz={r.z:.6f} expected {expect:.6f}")

    # Gripper: the trigger's analog pull, only while the clutch is closed.
    d.grips.clear()
    for _ in range(6):
        d.button(True, trigger=0.6)
        d.spin(0.05)
    check("gripper follows the trigger axis", d.grips and abs(d.grips[-1] - 0.6) < 1e-4,
          f"got {d.grips[-1] if d.grips else None}")

    d.button(False)
    d.spin(0.4)
    check("clutch opens on release", d.clutch[-1] is False, f"saw {d.clutch}")

    n_grip = len(d.grips)
    for _ in range(6):
        d.button(False, trigger=0.9)
        d.spin(0.05)
    check("no gripper commands while the clutch is open", len(d.grips) == n_grip,
          f"{len(d.grips) - n_grip} published after release")
    n = len(d.deltas)
    d.at = (1.0, 0.0, 1.5, 0.0)
    d.spin(0.4)
    check("no deltas while the clutch is open", len(d.deltas) == n,
          f"{len(d.deltas) - n} published after release")

    d.at = (1.0, 0.0, 1.0, 0.0)
    d.spin(0.3)
    d.button(True)
    d.spin(0.3)
    check("clutch closes again", d.clutch[-1] is True, f"saw {d.clutch}")

    d.streaming = False
    # Let a pose already in flight arrive before counting, but stay inside pose_timeout_s so
    # the clutch is still closed: the claim is about what follows the dropout, not precedes it.
    d.spin(0.15)
    before = len(d.deltas)
    d.spin(1.2)
    check("dropout opens the clutch", d.clutch[-1] is False, f"saw {d.clutch}")
    check("last delta is not repeated", len(d.deltas) == before,
          f"{len(d.deltas) - before} published after the poses stopped")

    d.destroy_node()
    rclpy.shutdown()
    print(f"\n{len(fails)} failed" if fails else "\nall passed")
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
