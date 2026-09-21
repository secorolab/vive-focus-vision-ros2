/* SPDX-License-Identifier: MIT */
#include "vive_vr_ros2/openarm_teleop.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace vive_vr_ros2 {
namespace {
template<typename T>
T parameter(rclcpp::Node &node, const std::string &name, const T &value)
{
    return node.has_parameter(name) ? node.get_parameter(name).get_value<T>()
                                    : node.declare_parameter<T>(name, value);
}

/* Past this the tool is not on its target yet, which a large hand movement legitimately is. */
constexpr double kOutOfReachPosition = 0.05; // [m]
constexpr double kOutOfReachRotation = 0.20; // [rad]

/* Only once the gap has stopped closing for this long has the arm actually run out of reach. */
constexpr double kProgressWindow = 0.5;      // [s]
constexpr double kProgressPosition = 1e-3;   // [m]
constexpr double kProgressRotation = 1e-2;   // [rad]

} // namespace

double OpenArmTeleop::age(Clock::time_point since)
{
    return std::chrono::duration<double>(Clock::now() - since).count();
}

OpenArmTeleop::OpenArmTeleop(rclcpp::Node &node, mjModel *model, mjData *data, const std::string &arm)
    : arm_(arm), node_(node), model_(model), data_(data), ik_(model, arm)
{
    const std::string pairing_prefix = arm == "right" ? "openarm." : "openarm.left.";
    scale_ = parameter<double>(node, "openarm.translation_scale", 0.25);
    radius_ = parameter<double>(node, "openarm.max_translation_m", 0.15);
    timeout_ = parameter<double>(node, "openarm.timeout_s", 0.25);
    finger_speed_ = parameter<double>(node, "openarm.finger_speed_m_s", 0.02);
    rotation_limit_ = parameter<double>(node, "openarm.max_rotation_rad", 1.75);
    alignment_tolerance_ = parameter<double>(node, "openarm.alignment_tolerance_rad", 0.20);
    position_tolerance_ = parameter<double>(node, "openarm.alignment_tolerance_m", 0.15);
    require_position_alignment_ = parameter<bool>(node, "openarm.require_position_alignment", false);
    alignment_configured_ = parameter<bool>(node, pairing_prefix + "alignment_configured", false);
    require_alignment_ = parameter<bool>(node, "openarm.require_alignment", true);
    gains_.joint_speed = parameter<double>(node, "openarm.joint_speed_rad_s", 1.5);
    gains_.max_linear = parameter<double>(node, "openarm.max_linear_speed_m_s", 0.5);
    gains_.max_angular = parameter<double>(node, "openarm.max_angular_speed_rad_s", 2.0);
    gains_.tracking_tau = parameter<double>(node, "openarm.tracking_time_constant_s", 0.12);
    gains_.damping = parameter<double>(node, "openarm.damping", 0.05);
    gains_.posture_gain = parameter<double>(node, "openarm.posture_gain_hz", 0.5);
    const auto pairing = parameter<std::vector<double>>(node,
      pairing_prefix + "controller_to_tool_xyzw", {0.0, 0.0, 0.0, 1.0});
    if (pairing.size() != 4) throw std::runtime_error("Invalid controller/tool pairing");
    pairing_q_[0] = pairing[3];
    for (int i = 0; i < 3; ++i) pairing_q_[i+1] = pairing[i];
    for (double v : pairing_q_) if (!std::isfinite(v)) throw std::runtime_error("Invalid pairing");
    if (std::abs(mju_norm(pairing_q_, 4) - 1.0) > 0.01) throw std::runtime_error("Invalid pairing norm");
    mju_normalize4(pairing_q_);
    for (double value : { scale_, radius_, timeout_, finger_speed_, rotation_limit_,
                          alignment_tolerance_, position_tolerance_,
                          gains_.joint_speed, gains_.max_linear,
                          gains_.max_angular, gains_.tracking_tau, gains_.damping }) {
        if (!std::isfinite(value) || value <= 0) throw std::runtime_error("Invalid OpenArm limits");
    }
    if (!std::isfinite(gains_.posture_gain) || gains_.posture_gain < 0) {
        throw std::runtime_error("Invalid OpenArm posture gain");
    }
    for (int i = 0; i < 2; ++i) {
        const auto name = "openarm_" + arm_ + "_finger_joint" + std::to_string(i + 1);
        const int joint = mj_name2id(model, mjOBJ_JOINT, name.c_str());
        finger_motors_[i] = mj_name2id(model, mjOBJ_ACTUATOR, (name + "_position").c_str());
        if (joint < 0 || finger_motors_[i] < 0) {
            throw std::runtime_error("Missing OpenArm finger actuator");
        }
        fingers_[i] = model->jnt_qposadr[joint];
    }
    const auto ns = node.get_parameter("out_ns").as_string() + "/teleop/" + arm_;
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
      node.get_parameter("out_ns").as_string() + "/sim/" + arm_ + "/ee_pose", rclcpp::SensorDataQoS());
    alignment_pose_pub_ = node.create_publisher<geometry_msgs::msg::PoseStamped>(
      ns + "/alignment_pose", rclcpp::SensorDataQoS());
    status_pub_ = node.create_publisher<vive_vr_ros2::msg::TeleopStatus>(ns + "/status", 1);
    controller_sub_ = node.create_subscription<geometry_msgs::msg::PoseStamped>(
      node.get_parameter("out_ns").as_string() + "/" + arm_ + "/pose", rclcpp::SensorDataQoS().keep_last(1),
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) { controller(*msg); });
    reset();
    RCLCPP_INFO(node.get_logger(), "%s-arm simulation: side grip enables motion; rear trigger controls gripper", arm_.c_str());
}

void OpenArmTeleop::controller(const geometry_msgs::msg::PoseStamped &msg)
{
    if (msg.header.frame_id != "world") {
        RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                             "%s controller pose is in frame '%s', not 'world'; ignored",
                             arm_.c_str(), msg.header.frame_id.c_str());
        return;
    }
    const auto &q = msg.pose.orientation;
    const auto &p = msg.pose.position;
    mjtNum quat[4] = {q.w, q.x, q.y, q.z};
    mjtNum point[3] = {p.x, p.y, p.z};
    for (double v : quat) if (!std::isfinite(v)) return;
    for (double v : point) if (!std::isfinite(v)) return;
    if (std::abs(mju_norm(quat, 4) - 1.0) > 0.1) return;
    mju_normalize4(quat);
    mju_copy4(controller_q_, quat);
    mju_copy3(controller_p_, point);
    controller_received_ = Clock::now();
    have_controller_ = true;
}

OpenArmTeleop::Match OpenArmTeleop::match()
{
    Match out;
    if (!have_controller_ || age(controller_received_) > timeout_) return out;
    mj_kinematics(model_, data_);
    mjtNum mapped[4], gap[3];
    mju_mulQuat(mapped, controller_q_, pairing_q_);
    const double dot = mju_dot(mapped, data_->xquat + 4 * ik_.tcp, 4);
    mju_sub3(gap, controller_p_, data_->xpos + 3 * ik_.tcp);
    out.rotation = 2 * std::acos(std::clamp(std::abs(dot), 0.0, 1.0));
    out.position = mju_norm3(gap);
    out.live = true;
    return out;
}

bool OpenArmTeleop::aligned()
{
    if (!alignment_configured_) return false;
    const Match m = match();
    return m.live && m.rotation <= alignment_tolerance_ &&
           (!require_position_alignment_ || m.position <= position_tolerance_);
}

void OpenArmTeleop::hold(const char *reason)
{
    stop_reason_ = reason;
    ik_.sync(data_);
    for (int i = 0; i < 7; ++i) data_->ctrl[ik_.motors[i]] = data_->qpos[ik_.qpos[i]];
    for (int i = 0; i < 2; ++i) data_->ctrl[finger_motors_[i]] = data_->qpos[fingers_[i]];
    finger_target_ = data_->qpos[fingers_[0]];
    active_ = false;
    have_target_ = false;
    have_trigger_ = false;
    tracking_ = {};
    stalled_ = false;
}

void OpenArmTeleop::reset()
{
    hold();
    released_ = false;
}

void OpenArmTeleop::clutch(bool down)
{
    if (!down) {
        if (active_) hold();
        stop_reason_.clear();
        released_ = true;
        return;
    }
    if (active_ || !released_) return;
    released_ = false;
    if (require_alignment_ && !aligned()) return;
    mj_forward(model_, data_);
    ik_.sync(data_);
    mju_copy3(anchor_p_, data_->xpos + 3 * ik_.tcp);
    mju_copy4(anchor_q_, data_->xquat + 4 * ik_.tcp);
    last_delta_ = Clock::now();
    active_ = true;
    have_target_ = false;
    have_trigger_ = false;
    tracking_ = {};
    stalled_ = false;
    progress_at_ = Clock::now();
    best_position_ = best_rotation_ = std::numeric_limits<double>::max();
}

void OpenArmTeleop::gripper(float value)
{
    if (!active_ || !have_target_ || !std::isfinite(value) || age(last_delta_) > timeout_) return;
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
    if (msg.header.frame_id != arm_ + "_tool_ref" || msg.child_frame_id != arm_ + "_tool_cmd") {
        RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                             "%s delta names frames '%s'->'%s'; ignored",
                             arm_.c_str(), msg.header.frame_id.c_str(), msg.child_frame_id.c_str());
        return;
    }
    const auto &t = msg.transform.translation;
    const auto &q = msg.transform.rotation;
    mjtNum p[3] = { t.x * scale_, t.y * scale_, t.z * scale_ };
    mjtNum quat[4] = { q.w, q.x, q.y, q.z };
    for (double value : p) if (!std::isfinite(value)) { hold("invalid_target"); return; }
    for (double value : quat) if (!std::isfinite(value)) { hold("invalid_target"); return; }
    const double norm = mju_norm(quat, 4);
    if (norm < 0.9 || norm > 1.1) { hold("invalid_target"); return; }
    mju_normalize4(quat);
    const double angle = 2 * std::acos(std::clamp(std::abs(quat[0]), 0.0, 1.0));
    /* The first delta of a press is against a reference captured at it: anything but a
     * near-identity one belongs to the previous press. */
    if (mju_norm3(p) > radius_ || angle > rotation_limit_ ||
        (!have_target_ && (mju_norm3(p) > 0.025 || angle > 0.15))) {
        hold(mju_norm3(p) > radius_ ? "translation_limit" :
             angle > rotation_limit_ ? "rotation_limit" : "reference_mismatch");
        RCLCPP_WARN(node_.get_logger(), "%s target exceeded motion bounds; release and re-grip", arm_.c_str());
        return;
    }
    mju_rotVecQuat(target_p_, p, anchor_q_);
    mju_addTo3(target_p_, anchor_p_);
    mju_mulQuat(target_q_, anchor_q_, quat);
    last_delta_ = Clock::now();
    have_target_ = true;
}

void OpenArmTeleop::tick(double dt)
{
    if (active_ && age(last_delta_) > timeout_) {
        hold("tracking_timeout");
        RCLCPP_WARN(node_.get_logger(), "%s tracking timeout: holding; release and re-grip", arm_.c_str());
    }
    if (active_ && have_target_) {
        tracking_ = ik_.step(target_p_, target_q_, dt, gains_);
        const auto &command = ik_.command();
        for (int i = 0; i < 7; ++i) data_->ctrl[ik_.motors[i]] = command[i];
        for (int i = 0; i < 2; ++i) {
            auto &finger = data_->ctrl[finger_motors_[i]];
            finger += std::clamp(finger_target_ - finger, -finger_speed_ * dt, finger_speed_ * dt);
        }
        if (tracking_.position_error <= kOutOfReachPosition &&
            tracking_.rotation_error <= kOutOfReachRotation) {
            stalled_ = false;
        } else if (tracking_.position_error < best_position_ - kProgressPosition ||
                   tracking_.rotation_error < best_rotation_ - kProgressRotation) {
            best_position_ = tracking_.position_error;
            best_rotation_ = tracking_.rotation_error;
            progress_at_ = Clock::now();
            stalled_ = false;
        } else if (age(progress_at_) > kProgressWindow && !stalled_) {
            stalled_ = true;
            RCLCPP_WARN(node_.get_logger(),
                        "%s tool has stopped closing on its target, %.0f mm and %.0f deg away; "
                        "the arm is as close as it can reach",
                        arm_.c_str(), tracking_.position_error * 1e3, tracking_.rotation_error * 180.0 / mjPI);
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
    const Match m = match();
    if (m.live && alignment_configured_) {
        // controller * pairing = tool, so the desired controller is tool * inverse(pairing).
        mjtNum inverse[4], wanted[4];
        mju_negQuat(inverse, pairing_q_);
        mju_mulQuat(wanted, data_->xquat + 4 * ik_.tcp, inverse);
        auto target = pose;
        target.pose.position.x = controller_p_[0];
        target.pose.position.y = controller_p_[1];
        target.pose.position.z = controller_p_[2];
        target.pose.orientation.w = wanted[0];
        target.pose.orientation.x = wanted[1];
        target.pose.orientation.y = wanted[2];
        target.pose.orientation.z = wanted[3];
        alignment_pose_pub_->publish(target);
    }
    vive_vr_ros2::msg::TeleopStatus status;
    status.header.stamp = pose.header.stamp;
    status.header.frame_id = "world";
    status.position_error_m = m.position;
    status.rotation_error_rad = m.rotation;
    // Zero signals relative-position engagement to the headset (no proximity gate).
    status.position_tolerance_m = require_position_alignment_ ? position_tolerance_ : 0.0;
    status.rotation_tolerance_rad = alignment_tolerance_;
    if (active_) status.state = !have_target_ ? "waiting_delta"
                              : stalled_      ? "unreachable"
                                              : "engaged";
    else if (!released_) status.state = "release_grip";
    else if (!alignment_configured_) status.state = "calibration_required";
    else if (!m.live) status.state = "tracking_lost";
    else status.state = aligned() ? "ready" : "align_pose";
    status.stop_reason = stop_reason_;
    status.ready = status.state == "ready";
    status_pub_->publish(status);
}

} // namespace vive_vr_ros2
