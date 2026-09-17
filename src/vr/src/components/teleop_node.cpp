/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Vamsi Kalagaturu
 * See LICENSE for details. */

/* How far the operator's hand has moved since the clutch was pressed, per arm, in the tool
 * frame. Nothing here knows what is on the other end: no IK, no joint limits, no robot.
 *
 * Deltas rather than absolute targets is what lets this run uncalibrated. If the unknown
 * play-space transform is R, every pose arrives as R*T, and the reference cancels it exactly:
 * (R*T_ref)^-1 * (R*T_now) = T_ref^-1 * T_now, orientation included. See docs/teleop.md. */

#include <algorithm>
#include <chrono>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/LinearMath/Vector3.hpp>

namespace vr {
namespace {

tf2::Transform to_tf(const geometry_msgs::msg::Pose &p)
{
    return tf2::Transform(
      tf2::Quaternion(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w),
      tf2::Vector3(p.position.x, p.position.y, p.position.z));
}

/** The arm names under `teleop.<arm>.<key>`, taken from what was actually passed in.
 *
 * The overrides come from the parameters interface rather than NodeOptions: a --params-file
 * lands in the former, while the latter holds only what a caller set in code.
 */
std::set<std::string> arm_names(const std::map<std::string, rclcpp::ParameterValue> &overrides)
{
    std::set<std::string> names;
    for (const auto &[n, unused] : overrides) {
        (void)unused;
        if (n.rfind("teleop.", 0) != 0) continue;
        const size_t arm_start = std::string("teleop.").size();
        const size_t arm_end   = n.find('.', arm_start);
        // teleop.pose_timeout_s is a scalar of this node, not an arm.
        if (arm_end == std::string::npos) continue;
        names.insert(n.substr(arm_start, arm_end - arm_start));
    }
    return names;
}

} // namespace

class TeleopNode : public rclcpp::Node
{
  public:
    explicit TeleopNode(const rclcpp::NodeOptions &options) : Node("vr_teleop", options)
    {
        out_ns_ = declare_parameter<std::string>("out_ns", "/vr");

        /* A clutch held perfectly still must stay closed, so a dropout is the absence of pose
         * messages and not <hand>/active, which reports "has not moved" and would open the
         * clutch on anyone holding a position. */
        pose_timeout_s_ = declare_parameter<double>("teleop.pose_timeout_s", 0.25);

        for (const std::string &name :
             arm_names(get_node_parameters_interface()->get_parameter_overrides())) {
            add_arm(name);
        }

        if (arms_.empty()) {
            RCLCPP_WARN(get_logger(), "no teleop.<arm>.* parameters; nothing to drive");
            return;
        }

        watchdog_ = create_wall_timer(std::chrono::milliseconds(50), [this] { check_stale(); });
    }

  private:
    struct Arm
    {
        std::string name;
        std::string hand;
        int         clutch_button = 1;

        /* Controller axes onto tool axes: OpenXR's grip pose puts -Z toward the thumb while a
         * tool frame puts +Z along the approach, so identity points the gripper backwards. A
         * constant of the controller-and-gripper pairing, not a per-session measurement. */
        tf2::Transform tool_R = tf2::Transform::getIdentity();

        /* The index finger's analog pull, 0 open to 1 closed. An axis rather than the button
         * so a partial grip survives the trip; a consumer wanting two states thresholds it. */
        int gripper_axis = 2;

        rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr delta_pub;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                  clutch_pub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr               gripper_pub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr   pose_sub;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr             joy_sub;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr   ee_sub;

        bool           have_pose = false;
        tf2::Transform pose      = tf2::Transform::getIdentity();
        rclcpp::Time   pose_stamp;

        bool           clutched = false;
        bool           pressed  = false;
        tf2::Transform ref      = tf2::Transform::getIdentity();
    };

    void add_arm(const std::string &name)
    {
        auto arm  = std::make_unique<Arm>();
        arm->name = name;

        const std::string prefix = "teleop." + name + ".";
        arm->hand = declare_parameter<std::string>(prefix + "hand", name);
        /* Stick click, not squeeze: squeeze is grab_button, and one press must not both grab a
         * simulated body and engage teleop. The trigger is the gripper. */
        arm->clutch_button = static_cast<int>(declare_parameter<int>(prefix + "clutch_button", 4));
        arm->gripper_axis  = static_cast<int>(declare_parameter<int>(prefix + "gripper_axis", 2));

        const std::string delta_topic = declare_parameter<std::string>(
          prefix + "delta_topic", out_ns_ + "/teleop/" + name + "/delta");
        const std::string clutch_topic = declare_parameter<std::string>(
          prefix + "clutch_topic", out_ns_ + "/teleop/" + name + "/clutch");
        const std::string gripper_topic = declare_parameter<std::string>(
          prefix + "gripper_topic", out_ns_ + "/teleop/" + name + "/gripper");
        const std::string ee_topic =
          declare_parameter<std::string>(prefix + "ee_pose_topic", "");

        const auto rpy = declare_parameter<std::vector<double>>(
          prefix + "tool_from_controller_rpy", { 0.0, 0.0, 0.0 });
        if (rpy.size() != 3) {
            throw std::runtime_error(prefix + "tool_from_controller_rpy must have 3 elements");
        }
        tf2::Quaternion q;
        q.setRPY(rpy[0] * M_PI / 180.0, rpy[1] * M_PI / 180.0, rpy[2] * M_PI / 180.0);
        arm->tool_R.setRotation(q);

        arm->delta_pub = create_publisher<geometry_msgs::msg::TransformStamped>(
          delta_topic, rclcpp::SensorDataQoS());
        /* Latched: a consumer that starts late must learn the clutch is open rather than wait
         * for a press that may not come. */
        arm->clutch_pub = create_publisher<std_msgs::msg::Bool>(
          clutch_topic, rclcpp::QoS(1).transient_local());
        arm->gripper_pub =
          create_publisher<std_msgs::msg::Float32>(gripper_topic, rclcpp::SensorDataQoS());

        Arm *a       = arm.get();
        arm->pose_sub = create_subscription<geometry_msgs::msg::PoseStamped>(
          out_ns_ + "/" + arm->hand + "/pose", rclcpp::SensorDataQoS(),
          [this, a](geometry_msgs::msg::PoseStamped::SharedPtr msg) { on_pose(*a, *msg); });
        arm->joy_sub = create_subscription<sensor_msgs::msg::Joy>(
          out_ns_ + "/" + arm->hand + "/joy", rclcpp::SensorDataQoS(),
          [this, a](sensor_msgs::msg::Joy::SharedPtr msg) { on_joy(*a, *msg); });

        /* Subscribed and unused. It is here so that absolute targets and drift detection do not
         * change the topic contract when they arrive; see docs/teleop.md. */
        if (!ee_topic.empty()) {
            arm->ee_sub = create_subscription<geometry_msgs::msg::PoseStamped>(
              ee_topic, rclcpp::SensorDataQoS(),
              [](geometry_msgs::msg::PoseStamped::SharedPtr) {});
        }

        publish_clutch(*a, false);
        RCLCPP_INFO(get_logger(),
                    "%s: %s hand, clutch button %d, gripper axis %d, tool rpy "
                    "[%.1f %.1f %.1f] deg -> %s",
                    name.c_str(), arm->hand.c_str(), arm->clutch_button, arm->gripper_axis,
                    rpy[0], rpy[1], rpy[2], delta_topic.c_str());
        arms_.push_back(std::move(arm));
    }

    void on_pose(Arm &arm, const geometry_msgs::msg::PoseStamped &msg)
    {
        arm.pose       = to_tf(msg.pose);
        arm.pose_stamp = rclcpp::Time(msg.header.stamp).nanoseconds() ? rclcpp::Time(msg.header.stamp)
                                                                     : now();
        arm.have_pose  = true;
        if (arm.clutched) { publish_delta(arm); }
    }

    void on_joy(Arm &arm, const sensor_msgs::msg::Joy &msg)
    {
        if (arm.clutch_button < 0 ||
            static_cast<size_t>(arm.clutch_button) >= msg.buttons.size()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "%s: button %d is outside a Joy of %zu buttons", arm.name.c_str(),
                                 arm.clutch_button, msg.buttons.size());
            return;
        }

        publish_gripper(arm, msg);

        const bool down = msg.buttons[static_cast<size_t>(arm.clutch_button)] != 0;
        if (down == arm.pressed) return;
        arm.pressed = down;

        if (!down) {
            close_clutch(arm, "released");
            return;
        }

        if (!arm.have_pose) {
            RCLCPP_WARN(get_logger(), "%s: clutch pressed before any pose arrived; ignored",
                        arm.name.c_str());
            return;
        }

        /* Every press takes a fresh reference, which is what makes re-clutching recover reach
         * the way lifting a mouse does: nothing accumulates across presses. */
        arm.ref      = arm.pose;
        arm.clutched = true;
        publish_clutch(arm, true);
    }

    /** Only while the clutch is closed: a disengaged operator must not be closing a real hand. */
    void publish_gripper(const Arm &arm, const sensor_msgs::msg::Joy &msg)
    {
        if (!arm.clutched) return;

        float value;
        if (arm.gripper_axis >= 0 &&
            static_cast<size_t>(arm.gripper_axis) < msg.axes.size()) {
            value = msg.axes[static_cast<size_t>(arm.gripper_axis)];
        } else if (!msg.buttons.empty()) {
            // No analog axis to read: the trigger's button is all there is.
            value = msg.buttons[0] != 0 ? 1.0f : 0.0f;
        } else {
            return;
        }

        std_msgs::msg::Float32 out;
        out.data = std::clamp(value, 0.0f, 1.0f);
        arm.gripper_pub->publish(out);
    }

    void publish_delta(const Arm &arm)
    {
        const tf2::Transform delta = arm.ref.inverse() * arm.pose;
        // A change of frame for a transform is a conjugation, not an offset.
        const tf2::Transform in_tool = arm.tool_R.inverse() * delta * arm.tool_R;

        geometry_msgs::msg::TransformStamped msg;
        msg.header.stamp    = arm.pose_stamp;
        msg.header.frame_id = arm.name + "_tool_ref";
        msg.child_frame_id  = arm.name + "_tool_cmd";
        msg.transform.translation.x = in_tool.getOrigin().x();
        msg.transform.translation.y = in_tool.getOrigin().y();
        msg.transform.translation.z = in_tool.getOrigin().z();
        msg.transform.rotation.x    = in_tool.getRotation().x();
        msg.transform.rotation.y    = in_tool.getRotation().y();
        msg.transform.rotation.z    = in_tool.getRotation().z();
        msg.transform.rotation.w    = in_tool.getRotation().w();
        arm.delta_pub->publish(msg);
    }

    void close_clutch(Arm &arm, const char *why)
    {
        if (!arm.clutched) return;
        arm.clutched = false;
        publish_clutch(arm, false);
        RCLCPP_INFO(get_logger(), "%s: clutch open (%s)", arm.name.c_str(), why);
    }

    void publish_clutch(const Arm &arm, bool closed)
    {
        std_msgs::msg::Bool msg;
        msg.data = closed;
        arm.clutch_pub->publish(msg);
    }

    /** Poses stopping arriving means the hand is gone; the last delta is never repeated. */
    void check_stale()
    {
        for (const auto &arm : arms_) {
            if (!arm->clutched) continue;
            if ((now() - arm->pose_stamp).seconds() < pose_timeout_s_) continue;
            arm->pressed = false;
            close_clutch(*arm, "no pose");
        }
    }

    std::string                       out_ns_;
    double                            pose_timeout_s_ = 0.25;
    std::vector<std::unique_ptr<Arm>> arms_;
    rclcpp::TimerBase::SharedPtr      watchdog_;
};

} // namespace vr

RCLCPP_COMPONENTS_REGISTER_NODE(vr::TeleopNode)
