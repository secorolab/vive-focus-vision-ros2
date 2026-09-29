#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Vamsi Kalagaturu
# See LICENSE for details.

"""Drives the simulated TIAGo round a world's patrol.yaml with pure pursuit on its wheels."""

import argparse
import json
import math
import os
import pathlib

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray

from vive_vr_ros2.msg import BodyPoses

CACHE = pathlib.Path(os.environ.get("VIVE_VR_CACHE", pathlib.Path.home() / ".cache/vive_vr_ros2"))

# TIAGo base, from tiago.xml: wheel centre height and the distance between the wheels.
WHEEL_RADIUS = 0.0985
WHEEL_SEPARATION = 0.4044
WHEEL_LIMIT = 5.0


class Patrol(Node):
    def __init__(self, args):
        super().__init__("tiago_patrol")
        patrol = yaml.safe_load((CACHE / "worlds" / args.world / "patrol.yaml").read_text())
        manifest = json.loads((CACHE / "scenes" / args.world / "manifest.json").read_text())
        names = [b["name"] for b in manifest["bodies"]]
        self.base = names.index(patrol["base_body"])
        self.path = [tuple(p) for p in patrol["waypoints"]]
        self.args = args
        self.segment = 0
        self.pose = None

        self.get_logger().info(
            f"{len(self.path)} waypoints; SceneNode needs "
            f"ctrl_actuators:=\"[{', '.join(patrol['actuators'])}]\"")
        self.pub = self.create_publisher(Float64MultiArray, args.ctrl_topic, 1)
        self.create_subscription(BodyPoses, args.poses_topic, self.on_poses,
                                 qos_profile_sensor_data)
        self.create_timer(1.0 / args.rate, self.step)

    def on_poses(self, msg: BodyPoses):
        for i, body in enumerate(msg.ids):
            if body == self.base:
                p, q = msg.poses[i].position, msg.poses[i].orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                self.pose = (p.x, p.y, yaw)

    def lookahead(self, x: float, y: float) -> tuple[float, float]:
        """The point one lookahead distance along the path past the robot's projection on it."""
        n = len(self.path)
        while True:
            (ax, ay), (bx, by) = self.path[self.segment], self.path[(self.segment + 1) % n]
            dx, dy = bx - ax, by - ay
            length = math.hypot(dx, dy)
            t = ((x - ax) * dx + (y - ay) * dy) / (length * length)
            if t < 1.0 and math.hypot(bx - x, by - y) > self.args.lookahead:
                break
            self.segment = (self.segment + 1) % n
        along = max(t, 0.0) * length + self.args.lookahead
        if along <= length:
            return ax + dx * along / length, ay + dy * along / length
        (cx, cy) = self.path[(self.segment + 2) % n]
        rest = along - length
        seg = math.hypot(cx - bx, cy - by)
        return bx + (cx - bx) * rest / seg, by + (cy - by) * rest / seg

    def step(self):
        if self.pose is None:
            return
        x, y, yaw = self.pose
        gx, gy = self.lookahead(x, y)
        alpha = math.atan2(gy - y, gx - x) - yaw
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))

        # Pure pursuit (Coulter, CMU-RI-TR-92-01); a goal behind turns on the spot, not via an arc.
        if abs(alpha) > math.radians(60):
            v, w = 0.0, math.copysign(self.args.turn_rate, alpha)
        else:
            v = self.args.speed
            w = v * 2.0 * math.sin(alpha) / self.args.lookahead

        left = (v - w * WHEEL_SEPARATION / 2) / WHEEL_RADIUS
        right = (v + w * WHEEL_SEPARATION / 2) / WHEEL_RADIUS
        scale = min(1.0, WHEEL_LIMIT / max(abs(left), abs(right), 1e-9))
        self.pub.publish(Float64MultiArray(data=[left * scale, right * scale]))


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--world", default="supermarket")
    p.add_argument("--speed", type=float, default=0.4, help="forward speed [m/s]")
    p.add_argument("--turn-rate", type=float, default=0.8, help="on-the-spot turn [rad/s]")
    p.add_argument("--lookahead", type=float, default=0.6, help="pure pursuit distance [m]")
    p.add_argument("--rate", type=float, default=20.0)
    p.add_argument("--ctrl-topic", default="/vive_scene/ctrl")
    p.add_argument("--poses-topic", default="/vive_vr/body_poses")
    args = p.parse_args()

    rclpy.init()
    node = Patrol(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
