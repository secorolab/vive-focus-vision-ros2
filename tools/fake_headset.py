"""Stands in for the Unity client: speaks the same rosbridge contract over the same socket.

Publishes /vr/right/{pose,joy} at 90 Hz and checks what the PC side sends back, so the whole
topic contract can be exercised before an APK exists.
"""
import asyncio
import json
import time

import websockets

URL = "ws://127.0.0.1:9090"
DURATION = 3.0

# A pose with three distinct components and a non-identity rotation: a transposed axis or a
# dropped sign in the conversion cannot survive this unnoticed.
POSE = dict(px=0.11, py=0.22, pz=0.33, qx=0.0, qy=0.0, qz=0.7071068, qw=0.7071068)
BUTTONS = [1, 0, 1, 0, 0, 1]  # trigger, squeeze, primary, secondary, stick_click, menu
AXES = [0.5, -0.25, 1.0, 0.0]


async def main():
    got_scene = None
    body_pose_count = 0
    pc_time_count = 0
    first_body_poses = None

    async with websockets.connect(URL, max_size=None) as ws:
        for topic, type_ in [
            ("/vr/right/pose", "geometry_msgs/msg/PoseStamped"),
            ("/vr/right/joy", "sensor_msgs/msg/Joy"),
        ]:
            await ws.send(json.dumps({"op": "advertise", "topic": topic, "type": type_}))
        for topic, type_ in [
            ("/vr/scene", "std_msgs/msg/String"),
            ("/vr/body_poses", "geometry_msgs/msg/PoseArray"),
            ("/vr/pc_time", "builtin_interfaces/msg/Time"),
        ]:
            await ws.send(json.dumps({"op": "subscribe", "topic": topic, "type": type_}))

        async def publish():
            period = 1.0 / 90.0
            deadline = time.monotonic() + DURATION
            n = 0
            while time.monotonic() < deadline:
                now = time.time()
                header = {
                    "stamp": {"sec": int(now), "nanosec": int((now % 1) * 1e9)},
                    "frame_id": "vr_origin",
                }
                await ws.send(json.dumps({
                    "op": "publish", "topic": "/vr/right/pose",
                    "msg": {"header": header, "pose": {
                        "position": {"x": POSE["px"], "y": POSE["py"], "z": POSE["pz"]},
                        "orientation": {"x": POSE["qx"], "y": POSE["qy"],
                                        "z": POSE["qz"], "w": POSE["qw"]}}}}))
                await ws.send(json.dumps({
                    "op": "publish", "topic": "/vr/right/joy",
                    "msg": {"header": header, "axes": AXES, "buttons": BUTTONS}}))
                n += 1
                await asyncio.sleep(period)
            return n

        async def receive():
            nonlocal got_scene, body_pose_count, pc_time_count, first_body_poses
            deadline = time.monotonic() + DURATION
            while time.monotonic() < deadline:
                try:
                    raw = await asyncio.wait_for(ws.recv(), timeout=deadline - time.monotonic())
                except (asyncio.TimeoutError, ValueError):
                    break
                env = json.loads(raw)
                if env.get("op") != "publish":
                    continue
                if env["topic"] == "/vr/scene":
                    got_scene = json.loads(env["msg"]["data"])
                elif env["topic"] == "/vr/body_poses":
                    body_pose_count += 1
                    if first_body_poses is None:
                        first_body_poses = env["msg"]["poses"]
                elif env["topic"] == "/vr/pc_time":
                    pc_time_count += 1

        sent, _ = await asyncio.gather(publish(), receive())

    print(f"sent      {sent} pose+joy pairs in {DURATION}s "
          f"({sent / DURATION:.1f} Hz)")
    print(f"body_poses {body_pose_count} msgs ({body_pose_count / DURATION:.1f} Hz)")
    print(f"pc_time    {pc_time_count} msgs ({pc_time_count / DURATION:.1f} Hz)")
    if got_scene:
        bodies = got_scene["manifest"]["bodies"]
        print(f"scene      url={got_scene['url']} bodies={[b['name'] for b in bodies]}")
    else:
        print("scene      NOT RECEIVED")
    if first_body_poses:
        print(f"pose[0]    {first_body_poses[0]['position']}")
        if len(first_body_poses) > 1:
            print(f"pose[1]    {first_body_poses[1]['position']}")


asyncio.run(main())
