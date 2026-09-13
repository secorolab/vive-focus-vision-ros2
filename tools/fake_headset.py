"""Stands in for the Unity client: speaks the same rosbridge contract over the same socket.

Publishes raw controller pose and buttons, and checks what the PC side sends back, so the whole
topic contract can be exercised without a headset. Nothing here is hardcoded to a host or a
topic: pass --host/--ns to match whatever the ROS side is configured with.
"""
import argparse
import asyncio
import json
import math
import time

import websockets

# A pose with three distinct components and a non-identity rotation: a transposed axis or a
# dropped sign in the conversion cannot survive this unnoticed.
POSE = dict(px=0.11, py=0.22, pz=0.33, qx=0.0, qy=0.0, qz=0.7071068, qw=0.7071068)
BUTTONS = [1, 0, 1, 0, 0, 1]  # trigger, squeeze, primary, secondary, stick_click, menu
AXES = [0.5, -0.25, 1.0, 0.0]


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--host", default="127.0.0.1", help="rosbridge host")
    p.add_argument("--port", type=int, default=9090)
    p.add_argument("--raw-ns", default="/vr/raw", help="namespace the client publishes into")
    p.add_argument("--out-ns", default="/vr", help="namespace the PC side publishes into")
    p.add_argument("--hand", default="right")
    p.add_argument("--seconds", type=float, default=3.0)
    p.add_argument("--rate", type=float, default=90.0)
    p.add_argument(
        "--still",
        action="store_true",
        help="hold the pose exactly still, which InputNode should report as inactive",
    )
    return p.parse_args()


async def main(args):
    url = f"ws://{args.host}:{args.port}"
    pose_topic = f"{args.raw_ns}/{args.hand}/pose"
    joy_topic = f"{args.raw_ns}/{args.hand}/joy"

    got_scene = None
    body_pose_count = 0
    pc_time_count = 0
    first_body_poses = None

    async with websockets.connect(url, max_size=None) as ws:
        joints_topic = f"{args.raw_ns}/{args.hand}/joints"
        gaze_topic = f"{args.raw_ns}/gaze"
        for topic, type_ in [
            (pose_topic, "geometry_msgs/msg/PoseStamped"),
            (joy_topic, "sensor_msgs/msg/Joy"),
            (joints_topic, "geometry_msgs/msg/PoseArray"),
            (gaze_topic, "vr/msg/EyeGaze"),
        ]:
            await ws.send(json.dumps({"op": "advertise", "topic": topic, "type": type_}))
        for topic, type_ in [
            (f"{args.out_ns}/scene", "std_msgs/msg/String"),
            (f"{args.out_ns}/body_poses", "geometry_msgs/msg/PoseArray"),
            (f"{args.out_ns}/pc_time", "builtin_interfaces/msg/Time"),
        ]:
            await ws.send(json.dumps({"op": "subscribe", "topic": topic, "type": type_}))

        async def publish():
            period = 1.0 / args.rate
            deadline = time.monotonic() + args.seconds
            n = 0
            while time.monotonic() < deadline:
                now = time.time()
                header = {
                    "stamp": {"sec": int(now), "nanosec": int((now % 1) * 1e9)},
                    "frame_id": "vr_origin",
                }
                # A constant pose is exactly what InputNode is meant to call inactive, so motion
                # is needed to exercise the other branch. --still holds it put on purpose.
                wobble = 0.0 if args.still else 0.05 * math.sin(time.monotonic() * 2.0)
                await ws.send(json.dumps({
                    "op": "publish", "topic": pose_topic,
                    "msg": {"header": header, "pose": {
                        "position": {"x": POSE["px"] + wobble, "y": POSE["py"], "z": POSE["pz"]},
                        "orientation": {"x": POSE["qx"], "y": POSE["qy"],
                                        "z": POSE["qz"], "w": POSE["qw"]}}}}))
                await ws.send(json.dumps({
                    "op": "publish", "topic": joy_topic,
                    "msg": {"header": header, "axes": AXES, "buttons": BUTTONS}}))

                # 26 joints laid out along +x so each TF frame is distinguishable, with the
                # wrist first: InputNode reparents them, so a wrong order shows up as a hand
                # folded into itself rather than as an error.
                joints = [{"position": {"x": 0.01 * j + wobble, "y": 0.0, "z": 0.0},
                           "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}
                          for j in range(26)]
                await ws.send(json.dumps({
                    "op": "publish", "topic": joints_topic,
                    "msg": {"header": header, "poses": joints}}))

                eye = {"position": {"x": 0.03, "y": 1.6, "z": 0.0},
                       "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}}
                await ws.send(json.dumps({
                    "op": "publish", "topic": gaze_topic,
                    "msg": {"header": header, "left": eye, "right": eye,
                            "left_valid": True, "right_valid": True,
                            "left_pupil_diameter_mm": 3.4, "right_pupil_diameter_mm": 3.6,
                            "left_pupil_valid": True, "right_pupil_valid": True}}))
                n += 1
                await asyncio.sleep(period)
            return n

        async def receive():
            nonlocal got_scene, body_pose_count, pc_time_count, first_body_poses
            deadline = time.monotonic() + args.seconds
            while time.monotonic() < deadline:
                try:
                    raw = await asyncio.wait_for(ws.recv(), timeout=deadline - time.monotonic())
                except (asyncio.TimeoutError, ValueError):
                    break
                env = json.loads(raw)
                if env.get("op") != "publish":
                    continue
                topic = env["topic"]
                if topic.endswith("/scene"):
                    got_scene = json.loads(env["msg"]["data"])
                elif topic.endswith("/body_poses"):
                    body_pose_count += 1
                    if first_body_poses is None:
                        first_body_poses = env["msg"]["poses"]
                elif topic.endswith("/pc_time"):
                    pc_time_count += 1

        sent, _ = await asyncio.gather(publish(), receive())

    print(f"sent       {sent} pose+joy pairs in {args.seconds}s ({sent / args.seconds:.1f} Hz)"
          f"{' (still)' if args.still else ''}")
    print(f"body_poses {body_pose_count} msgs ({body_pose_count / args.seconds:.1f} Hz)")
    print(f"pc_time    {pc_time_count} msgs ({pc_time_count / args.seconds:.1f} Hz)")
    if got_scene:
        bodies = got_scene["manifest"]["bodies"]
        print(f"scene      url={got_scene['url']} bodies={[b['name'] for b in bodies]}")
    else:
        print("scene      NOT RECEIVED")
    if first_body_poses:
        for i, p in enumerate(first_body_poses[:2]):
            print(f"pose[{i}]    {p['position']}")


asyncio.run(main(parse_args()))
