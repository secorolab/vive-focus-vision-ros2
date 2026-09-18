/* SPDX-License-Identifier: MIT */
#pragma once

#include <chrono>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vive_vr_ros2/msg/teleop_status.hpp>
#include "vive_vr_ros2/openarm_ik.hpp"

namespace vive_vr_ros2 {

/** Consumes the existing teleop contract and commands only the owned MuJoCo simulation.
 * All callbacks and tick() run in the owner's mutually-exclusive callback group.
 * Liveness is when a message arrived, never its stamp: the headset's clock estimate steps. */
class OpenArmTeleop
{
  public:
    OpenArmTeleop(rclcpp::Node &node, mjModel *model, mjData *data);
    void tick(double dt);
    void reset();

  private:
    using Clock = std::chrono::steady_clock;

    /** How far the controller is from the tool pose it has to match before a press will take. */
    struct Match
    {
        double position = 0;   // [m]
        double rotation = 0;   // [rad]
        bool   live = false;   // a fresh controller pose exists to measure against
    };

    static double age(Clock::time_point since);
    Match match();
    bool aligned();
    void controller(const geometry_msgs::msg::PoseStamped &msg);
    void hold();
    void clutch(bool pressed);
    void delta(const geometry_msgs::msg::TransformStamped &msg);
    void gripper(float value);

    rclcpp::Node &node_;
    mjModel *model_;
    mjData *data_;
    OpenArmIk ik_;
    OpenArmIk::Gains gains_;
    OpenArmIk::Result tracking_;
    Clock::time_point progress_at_;
    double best_position_ = 0, best_rotation_ = 0;
    bool stalled_ = false;
    bool released_ = false;
    bool active_ = false;
    bool have_target_ = false;
    bool have_trigger_ = false;
    double last_trigger_ = 0.0;
    Clock::time_point last_delta_;
    double scale_, radius_, timeout_, finger_speed_, rotation_limit_;
    bool alignment_configured_, require_alignment_, have_controller_ = false;
    double alignment_tolerance_, position_tolerance_;
    mjtNum pairing_q_[4], controller_q_[4], controller_p_[3];
    Clock::time_point controller_received_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr controller_sub_;
    rclcpp::Publisher<vive_vr_ros2::msg::TeleopStatus>::SharedPtr status_pub_;
    mjtNum anchor_p_[3], anchor_q_[4], target_p_[3], target_q_[4];
    std::array<int, 2> fingers_, finger_motors_;
    double finger_target_ = 0.0;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr clutch_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr delta_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gripper_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pub_;
};

} // namespace vive_vr_ros2
