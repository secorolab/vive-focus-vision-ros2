#!/usr/bin/env python3
"""Record aligned controller, desired-tool and measured-tool snapshots. No motion commands."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--domain-id',type=int,default=84)
    parser.add_argument('--seconds', type=float, default=600)
    parser.add_argument('--mode', choices=['trajectory', 'stream'], default='trajectory')
    args = parser.parse_args()
    if not 0 < args.seconds <= 3600:
        parser.error('--seconds must be in (0, 3600]')
    if not 0 <= args.domain_id <= 232:
        parser.error('--domain-id must be between 0 and 232')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    output = args.output or Path('/tmp') / ('openarm-motion-' + datetime.now().strftime('%Y%m%d-%H%M%S-%f') + '.jsonl')
    rclpy.init()
    node = rclpy.create_node('openarm_motion_recorder')
    counts = {'right': 0, 'left': 0}
    try:
        with output.open('x') as log:
            log.write(json.dumps({'event': 'start', 'mode_label': args.mode, 'unix_time': time.time(),
                                  'note': 'Latest-sample snapshots in robot world frame; controller_age_s is receive age, not source-clock synchronization.'}) + '\n')
            def receive(msg, arm):
                try:
                    row = json.loads(msg.data)
                except ValueError:
                    return
                row.update(arm=arm, received_unix_s=time.time())
                log.write(json.dumps(row, allow_nan=False) + '\n')
                counts[arm] += 1
            for arm in counts:
                node.create_subscription(String, f'/vive_vr/teleop/{arm}/motion_diagnostics',
                                         lambda msg, side=arm: receive(msg, side), qos_profile_sensor_data)
            print(f'Motion diagnostic log: {output}\nWaiting for engaged arm movement. Ctrl+C stops logging only.', flush=True)
            end = time.monotonic() + args.seconds
            report = time.monotonic() + 5
            while rclpy.ok() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
                if time.monotonic() >= report:
                    log.flush()
                    print(f"Samples: right {counts['right']}, left {counts['left']}", flush=True)
                    report = time.monotonic() + 5
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    print(f'Saved diagnostic capture: {output}; samples: {counts}', flush=True)


if __name__ == '__main__':
    main()
