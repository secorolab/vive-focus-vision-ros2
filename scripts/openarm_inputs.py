#!/usr/bin/env python3
"""Read controller inputs without sending commands. Ctrl+C exits."""

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Joy


def main():
    rclpy.init()
    node = rclpy.create_node('openarm_input_check')
    previous = None

    def on_joy(msg):
        nonlocal previous
        if len(msg.axes) < 4 or len(msg.buttons) < 2:
            node.get_logger().warning('Unexpected Joy layout: check the headset APK version')
            return
        values = (round(msg.axes[2], 2), round(msg.axes[3], 2),
                  msg.buttons[0], msg.buttons[1])
        if values != previous:
            print(f'rear trigger: {values[0]:.2f} (button {values[2]}) | '
                  f'side grip: {values[1]:.2f} (button {values[3]})', flush=True)
            previous = values

    node.create_subscription(Joy, '/vive_vr/right/joy', on_joy, qos_profile_sensor_data)
    print('Press only the rear index trigger, then only the side grip. No commands are sent.', flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
