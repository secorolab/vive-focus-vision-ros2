/* SPDX-License-Identifier: MIT */
#include "vive_vr_ros2/openarm_teleop.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace vive_vr_ros2 {

OpenArmTeleop::OpenArmTeleop(rclcpp::Node &node, mjModel *model, mjData *data)
    : node_(node), model_(model), data_(data), ik_(model)
{
    scale_ = node.declare_parameter<double>("openarm.translation_scale", 0.25);
    radius_ = node.declare_parameter<double>("openarm.max_translation_m", 0.15);
    timeout_ = node.declare_parameter<double>("openarm.timeout_s", 0.25);
    speed_ = node.declare_parameter<double>("openarm.joint_speed_rad_s", 0.35);
    finger_speed_ = node.declare_parameter<double>("openarm.finger_speed_m_s", 0.02);
    rotation_limit_ = node.declare_parameter<double>("openarm.max_rotation_rad", 1.75);
    alignment_tolerance_ = node.declare_parameter<double>("openarm.alignment_tolerance_rad", 0.20);
    alignment_configured_ = node.declare_parameter<bool>("openarm.alignment_configured", false);
    require_alignment_ = node.declare_parameter<bool>("openarm.require_alignment", true);
    const auto pairing = node.declare_parameter<std::vector<double>>(
      "openarm.controller_to_tool_xyzw", {0.0, 0.0, 0.0, 1.0});
    if (pairing.size() != 4) throw std::runtime_error("Invalid controller/tool pairing");
    pairing_q_[0] = pairing[3];
    for (int i = 0; i < 3; ++i) pairing_q_[i+1] = pairing[i];
    for (double v : pairing_q_) if (!std::isfinite(v)) throw std::runtime_error("Invalid pairing");
    if (std::abs(mju_norm(pairing_q_, 4) - 1.0) > 0.01) throw std::runtime_error("Invalid pairing norm");
    mju_normalize4(pairing_q_);
    for (double value : { scale_, radius_, timeout_, speed_, finger_speed_, rotation_limit_, alignment_tolerance_ }) {
        if (!std::isfinite(value) || value <= 0) throw std::runtime_error("Invalid OpenArm limits");
    }
    for (int i = 0; i < 2; ++i) {
        const auto name = "openarm_right_finger_joint" + std::to_string(i + 1);
        const int joint = mj_name2id(model, mjOBJ_JOINT, name.c_str());
        finger_motors_[i] = mj_name2id(model, mjOBJ_ACTUATOR, (name + "_position").c_str());
        if (joint < 0 || finger_motors_[i] < 0) {
            throw std::runtime_error("Missing OpenArm finger actuator");
        }
        fingers_[i] = model->jnt_qposadr[joint];
    }
    const auto ns = node.get_parameter("out_ns").as_string() + "/teleop/right";
    clutch_sub_ = node.create_subscription<std_msgs::msg::Bool>(
      ns + "/clutch", rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::Bool::SharedPtr msg) { clutch(msg->data); });
    delta_sub_ = node.create_subscription<geometry_msgs::msg::TransformStamped>(
      ns + "/delta", rclcpp::SensorDataQoS().keep_last(1),
      [this](geometry_msgs::msg::TransformStamped::SharedPtr msg) { delta(*msg); });
    gripper_sub_ = node.create_subscription<std_msgs::msg::Float32>(
      ns + "/gripper", rclcpp::SensorDataQoS().keep_last(1),
      [this](std_msgs::msg::Float32::SharedPtr msg) { gripper(msg->data); });
    ee_pub_ = node.create_publisher<geometry_msgs::msg::PoseStamped>(
      node.get_parameter("out_ns").as_string() + "/sim/right/ee_pose", rclcpp::SensorDataQoS());
    status_pub_ = node.create_publisher<std_msgs::msg::String>(ns + "/status", 1);
    controller_sub_ = node.create_subscription<geometry_msgs::msg::PoseStamped>(
      node.get_parameter("out_ns").as_string() + "/right/pose", rclcpp::SensorDataQoS().keep_last(1),
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) { controller(*msg); });
    reset();
    RCLCPP_INFO(node.get_logger(), "Right-arm simulation: side grip enables motion; rear trigger controls gripper");
}

void OpenArmTeleop::controller(const geometry_msgs::msg::PoseStamped &msg)
{
    const int64_t stamp = rclcpp::Time(msg.header.stamp).nanoseconds();
    const double age = (node_.now().nanoseconds() - stamp) * 1e-9;
    have_controller_ = false;
    if (msg.header.frame_id != "world" || stamp <= controller_stamp_ || age < -0.05 || age > timeout_) return;
    const auto &q = msg.pose.orientation;
    mjtNum quat[4] = {q.w, q.x, q.y, q.z};
    for (double v : quat) if (!std::isfinite(v)) return;
    if (std::abs(mju_norm(quat, 4) - 1.0) > 0.1) return;
    mju_normalize4(quat);
    mju_copy4(controller_q_, quat);
    controller_stamp_ = stamp;
    controller_received_ = Clock::now();
    have_controller_ = true;
}

bool OpenArmTeleop::aligned()
{
    if (!alignment_configured_ || !have_controller_ ||
        std::chrono::duration<double>(Clock::now() - controller_received_).count() > timeout_) return false;
    mj_kinematics(model_, data_);
    mjtNum mapped[4];
    mju_mulQuat(mapped, controller_q_, pairing_q_);
    const double dot = mju_dot(mapped, data_->xquat + 4 * ik_.tcp, 4);
    return 2 * std::acos(std::clamp(std::abs(dot), 0.0, 1.0)) <= alignment_tolerance_;
}

void OpenArmTeleop::hold()
{
    for (int i = 0; i < 7; ++i) data_->ctrl[ik_.motors[i]] = data_->qpos[ik_.qpos[i]];
    for (int i = 0; i < 2; ++i) data_->ctrl[finger_motors_[i]] = data_->qpos[fingers_[i]];
    finger_target_ = data_->qpos[fingers_[0]];
    active_ = false;
    have_target_ = false;
    have_trigger_ = false;
}

void OpenArmTeleop::reset()
{
    hold();
    released_ = false;
    last_stamp_ = 0;
}

void OpenArmTeleop::clutch(bool down)
{
    if (!down) {
        if (active_) hold();
        released_ = true;
        return;
    }
    if (active_ || !released_) return;
    released_ = false;
    if (require_alignment_ && !aligned()) return;
    ik_reachable_ = true;
    mj_forward(model_, data_);
    mju_copy3(anchor_p_, data_->xpos + 3 * ik_.tcp);
    mju_copy4(anchor_q_, data_->xquat + 4 * ik_.tcp);
    press_stamp_ = node_.now().nanoseconds();
    last_stamp_ = 0;
    last_delta_ = Clock::now();
    active_ = true;
    have_target_ = false;
    have_trigger_ = false;
}

void OpenArmTeleop::gripper(float value)
{
    if (!active_ || !have_target_ || !std::isfinite(value) ||
        std::chrono::duration<double>(Clock::now() - last_delta_).count() > timeout_) return;
    const double trigger = std::clamp(double(value), 0.0, 1.0);
    // Re-engaging the clutch must not apply the resting trigger value and open a held object.
    // Capture the first sample, then follow actual trigger changes (not grip-button events).
    if (!have_trigger_) {
        last_trigger_ = trigger;
        have_trigger_ = true;
        return;
    }
    if (std::abs(trigger - last_trigger_) < 0.01) return;
    last_trigger_ = trigger;
    finger_target_ = 0.044 * (1.0 - trigger);
}

void OpenArmTeleop::delta(const geometry_msgs::msg::TransformStamped &msg)
{
    if (!active_) return;
    const int64_t stamp = rclcpp::Time(msg.header.stamp).nanoseconds();
    const double age = (node_.now().nanoseconds() - stamp) * 1e-9;
    if (msg.header.frame_id != "right_tool_ref" || msg.child_frame_id != "right_tool_cmd" ||
        stamp <= last_stamp_ || stamp < press_stamp_ - int64_t(timeout_ * 1e9) ||
        age < -0.05 || age > timeout_) return;
    const auto &t = msg.transform.translation;
    const auto &q = msg.transform.rotation;
    mjtNum p[3] = { t.x * scale_, t.y * scale_, t.z * scale_ };
    mjtNum quat[4] = { q.w, q.x, q.y, q.z };
    for (double value : p) if (!std::isfinite(value)) { hold(); return; }
    for (double value : quat) if (!std::isfinite(value)) { hold(); return; }
    const double norm = mju_norm(quat, 4);
    if (norm < 0.9 || norm > 1.1) { hold(); return; }
    mju_normalize4(quat);
    const double angle = 2 * std::acos(std::clamp(std::abs(quat[0]), 0.0, 1.0));
    if (mju_norm3(p) > radius_ || angle > rotation_limit_ ||
        (!have_target_ && (mju_norm3(p) > 0.025 || angle > 0.15))) {
        hold();
        RCLCPP_WARN(node_.get_logger(), "Right target exceeded motion bounds; release and re-grip");
        return;
    }
    mju_rotVecQuat(target_p_, p, anchor_q_);
    mju_addTo3(target_p_, anchor_p_);
    mju_mulQuat(target_q_, anchor_q_, quat);
    last_delta_ = Clock::now();
    last_stamp_ = stamp;
    have_target_ = true;
}

void OpenArmTeleop::tick(double dt)
{
    if (active_ && std::chrono::duration<double>(Clock::now() - last_delta_).count() > timeout_) {
        hold();
        RCLCPP_WARN(node_.get_logger(), "Right tracking timeout: holding; release and re-grip");
    }
    if (active_ && have_target_) {
        std::array<double, 7> solution;
        ik_reachable_ = ik_.solve(data_, target_p_, target_q_, solution);
        if (ik_reachable_) {
            for (int i = 0; i < 7; ++i) {
                auto &command = data_->ctrl[ik_.motors[i]];
                const double position = data_->qpos[ik_.qpos[i]];
                const double goal = std::clamp(solution[i], position - 0.10, position + 0.10);
                command += std::clamp(goal - command, -speed_ * dt, speed_ * dt);
            }
        } else {
            for (int i = 0; i < 7; ++i) data_->ctrl[ik_.motors[i]] = data_->qpos[ik_.qpos[i]];
            RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
                                "Right IK target unreachable: holding arm; move closer or re-grip");
        }
        // Gripper motion is independent of whether the arm's target is reachable.
        for (int i = 0; i < 2; ++i) {
            auto &command = data_->ctrl[finger_motors_[i]];
            command += std::clamp(finger_target_ - command, -finger_speed_ * dt, finger_speed_ * dt);
        }
    }
    mj_kinematics(model_, data_);
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = node_.now();
    pose.header.frame_id = "world";
    const auto *p = data_->xpos + 3 * ik_.tcp;
    const auto *q = data_->xquat + 4 * ik_.tcp;
    pose.pose.position.x = p[0];
    pose.pose.position.y = p[1];
    pose.pose.position.z = p[2];
    pose.pose.orientation.w = q[0];
    pose.pose.orientation.x = q[1];
    pose.pose.orientation.y = q[2];
    pose.pose.orientation.z = q[3];
    ee_pub_->publish(pose);
    std_msgs::msg::String status;
    if (active_) status.data = !have_target_ ? "waiting_delta" : ik_reachable_ ? "engaged" : "unreachable";
    else if (!released_) status.data = "release_grip";
    else if (!alignment_configured_) status.data = "calibration_required";
    else if (!have_controller_ || std::chrono::duration<double>(Clock::now()-controller_received_).count() > timeout_)
        status.data = "tracking_lost";
    else status.data = aligned() ? "ready" : "align_orientation";
    status_pub_->publish(status);
}

} // namespace vive_vr_ros2
