"""ROS 2 subscriptions only. No publishers, services, CAN or motion commands."""
import math
import threading
import time


def start(signals):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import JointState, Joy
    from std_msgs.msg import Bool
    from geometry_msgs.msg import PoseStamped
    from controller_manager_msgs.srv import ListControllers
    from control_msgs.msg import JointTrajectoryControllerState

    rclpy.init()
    node = Node('openarm_episode_observer')
    joints = [f'openarm_right_joint{i}' for i in range(1, 8)]
    finger = ['openarm_right_finger_joint1']

    def receive(key, names, values, stamp, expected):
        age = (node.get_clock().now().nanoseconds - (stamp.sec*10**9+stamp.nanosec))/1e9
        if len(names) != len(values) or len(set(names)) != len(names) or not -.05 <= age <= .15:
            return
        mapping = dict(zip(names, values))
        if not all(name in mapping and math.isfinite(mapping[name]) for name in expected): return
        signals.add(key, time.monotonic()-max(age, 0), [mapping[name] for name in expected])

    node.create_subscription(JointState, '/joint_states',
        lambda m: receive('state', m.name, m.position, m.header.stamp, joints+finger), qos_profile_sensor_data)
    for key, controller, names in [('arm', 'right_joint_trajectory_controller', joints),
                                   ('gripper', 'right_gripper_controller', finger)]:
        node.create_subscription(JointTrajectoryControllerState, f'/{controller}/controller_state',
            lambda m, k=key, n=names: receive(k, m.joint_names, m.reference.positions, m.header.stamp, n),
            qos_profile_sensor_data)

    def health(msg):
        with signals.lock: signals.health = (time.monotonic(), bool(msg.data))

    def joy(msg):
        age = (node.get_clock().now().nanoseconds - (msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))/1e9
        valid = -.05 <= age <= .25 and len(msg.buttons)>6 and msg.buttons[6] == 1
        with signals.lock: signals.world_lock = (time.monotonic(), valid)

    node.create_subscription(Bool, '/vive_vr/hardware/right/healthy', health,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(Joy, '/vive_vr/raw/right/joy', joy, qos_profile_sensor_data)
    def pose(msg):
        age = (node.get_clock().now().nanoseconds - (msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))/1e9
        q = msg.pose.orientation
        p = msg.pose.position
        if -.05 <= age <= .25 and all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w)) and .81 < q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w < 1.21:
            with signals.lock: signals.tracking = time.monotonic()-max(age, 0)
    node.create_subscription(PoseStamped, '/vive_vr/raw/right/pose', pose, qos_profile_sensor_data)
    client = node.create_client(ListControllers, '/controller_manager/list_controllers')
    pending = [None]
    def check_controllers():
        if pending[0] is not None and not pending[0].done(): return
        if not client.service_is_ready(): return
        pending[0] = client.call_async(ListControllers.Request())
        def done(future):
            try:
                active = {c.name for c in future.result().controller if c.state == 'active'}
                okay = {'right_joint_trajectory_controller','right_gripper_controller'} <= active
                with signals.lock: signals.controllers = (time.monotonic(), okay)
            except Exception:
                with signals.lock: signals.controllers = (time.monotonic(), False)
        pending[0].add_done_callback(done)
    node.create_timer(.5, check_controllers)
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    def close():
        rclpy.shutdown()
        thread.join(timeout=3)
        node.destroy_node()
    return close
