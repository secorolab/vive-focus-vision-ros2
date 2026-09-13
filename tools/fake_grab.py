"""Drives a pinch at a body and checks the simulation actually lifts it.

Publishes calibrated hand joints straight into <out_ns>, which is what the grabber consumes, so
it exercises the grab path without a headset or the input node.
"""
import argparse
import json
import math
import time

import websockets
import asyncio

JOINTS = 26
THUMB_TIP, INDEX_TIP, WRIST = 5, 10, 0


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=9090)
    p.add_argument("--out-ns", default="/vr")
    p.add_argument("--hand", default="right")
    p.add_argument("--target", nargs=3, type=float, default=[0.3, 0.0, 0.05],
                   help="where the object starts, i.e. where to pinch")
    p.add_argument("--lift", type=float, default=0.35, help="how high to raise it [m]")
    p.add_argument("--seconds", type=float, default=8.0)
    p.add_argument("--rate", type=float, default=60.0)
    p.add_argument("--body", type=int, default=1,
                   help="index into body_poses to watch; matches the manifest order")
    return p.parse_args()


def hand_msg(now, centre, gap):
    """A hand whose thumb and index straddle `centre`, `gap` apart."""
    poses = []
    for j in range(JOINTS):
        p = list(centre)
        if j == THUMB_TIP:
            p[1] -= gap / 2
        elif j == INDEX_TIP:
            p[1] += gap / 2
        poses.append({"position": {"x": p[0], "y": p[1], "z": p[2]},
                      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}})
    return {"header": {"stamp": {"sec": int(now), "nanosec": int((now % 1) * 1e9)},
                       "frame_id": "world"},
            "poses": poses}


async def main(args):
    url = f"ws://{args.host}:{args.port}"
    joints_topic = f"{args.out_ns}/{args.hand}/joints"
    body_topic = f"{args.out_ns}/body_poses"
    heights = []

    async with websockets.connect(url, max_size=None) as ws:
        await ws.send(json.dumps({"op": "advertise", "topic": joints_topic,
                                  "type": "geometry_msgs/msg/PoseArray"}))
        await ws.send(json.dumps({"op": "subscribe", "topic": body_topic,
                                  "type": "geometry_msgs/msg/PoseArray"}))

        async def publish():
            period = 1.0 / args.rate
            deadline = time.monotonic() + args.seconds
            start = time.monotonic()
            while time.monotonic() < deadline:
                t = time.monotonic() - start
                # Approach open-handed, pinch, then raise straight up.
                if t < 1.0:
                    centre, gap = args.target, 0.08
                elif t < 1.5:
                    centre, gap = args.target, 0.01
                else:
                    climb = min(args.lift, (t - 1.5) * 0.15)
                    centre = [args.target[0], args.target[1], args.target[2] + climb]
                    gap = 0.01
                await ws.send(json.dumps({"op": "publish", "topic": joints_topic,
                                          "msg": hand_msg(time.time(), centre, gap)}))
                await asyncio.sleep(period)

        async def receive():
            deadline = time.monotonic() + args.seconds
            while time.monotonic() < deadline:
                try:
                    raw = await asyncio.wait_for(ws.recv(),
                                                 timeout=max(0.01, deadline - time.monotonic()))
                except (asyncio.TimeoutError, ValueError):
                    break
                env = json.loads(raw)
                if env.get("op") == "publish" and env["topic"].endswith("/body_poses"):
                    poses = env["msg"]["poses"]
                    if len(poses) > args.body:
                        heights.append((time.monotonic(), poses[args.body]["position"]["z"]))

        await asyncio.gather(publish(), receive())

    if not heights:
        print("no body poses received")
        return
    t0 = heights[0][0]
    start_z = heights[0][1]
    peak_z = max(z for _, z in heights)
    end_z = heights[-1][1]
    print(f"samples {len(heights)} over {heights[-1][0] - t0:.1f}s")
    print(f"start z {start_z:.3f}  peak z {peak_z:.3f}  end z {end_z:.3f}")
    print("LIFTED" if peak_z > start_z + 0.05 else "NOT LIFTED")


asyncio.run(main(parse_args()))
