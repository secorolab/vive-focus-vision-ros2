/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Vamsi Kalagaturu
 * See LICENSE for details. */

#include "vr/grabber.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace vr {
namespace {

void clamp3(mjtNum v[3], double limit)
{
    const double n = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (n > limit && n > 0.0) {
        const double s = limit / n;
        v[0] *= s;
        v[1] *= s;
        v[2] *= s;
    }
}

} // namespace

Grabber::Grabber(rclcpp::Node &node, const mjModel *model, GrabConf conf)
  : conf_(std::move(conf)), model_(model), node_(node)
{
    for (const std::string &hand : conf_.hands) {
        hands_[hand] = Hand{};
        if (conf_.use_controllers) {
            subs_.push_back(node.create_subscription<geometry_msgs::msg::PoseStamped>(
              conf_.topic_ns + "/" + hand + "/pose", rclcpp::SensorDataQoS(),
              [this, hand](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                  on_pose(hand, *msg);
              }));
            subs_.push_back(node.create_subscription<sensor_msgs::msg::Joy>(
              conf_.topic_ns + "/" + hand + "/joy", rclcpp::SensorDataQoS(),
              [this, hand](sensor_msgs::msg::Joy::SharedPtr msg) { on_joy(hand, *msg); }));
        }
        if (conf_.use_hands) {
            subs_.push_back(node.create_subscription<geometry_msgs::msg::PoseArray>(
              conf_.topic_ns + "/" + hand + "/joints", rclcpp::SensorDataQoS(),
              [this, hand](geometry_msgs::msg::PoseArray::SharedPtr msg) {
                  on_joints(hand, *msg);
              }));
        }
    }

    RCLCPP_INFO(node.get_logger(), "grab sources:%s%s, reach %.0f mm",
                conf_.use_controllers ? " controller button" : "",
                conf_.use_hands ? " pinch" : "", conf_.reach_m * 1000.0);
}

void Grabber::on_pose(const std::string &hand, const geometry_msgs::msg::PoseStamped &msg)
{
    Hand &h     = hands_[hand];
    h.pos[0]    = msg.pose.position.x;
    h.pos[1]    = msg.pose.position.y;
    h.pos[2]    = msg.pose.position.z;
    h.quat[0]   = msg.pose.orientation.w; // MuJoCo order is (w, x, y, z)
    h.quat[1]   = msg.pose.orientation.x;
    h.quat[2]   = msg.pose.orientation.y;
    h.quat[3]   = msg.pose.orientation.z;
    h.have_pose = true;
}

void Grabber::on_joy(const std::string &hand, const sensor_msgs::msg::Joy &msg)
{
    if (conf_.grab_button < 0 || conf_.grab_button >= static_cast<int>(msg.buttons.size())) return;
    hands_[hand].pressed = msg.buttons[conf_.grab_button] != 0;
}

/* Indices into the XRHandJointID order the client publishes; see the table in input_node.cpp. */
constexpr size_t kWrist    = 0;
constexpr size_t kThumbTip = 5;
constexpr size_t kIndexTip = 10;

void Grabber::on_joints(const std::string &hand, const geometry_msgs::msg::PoseArray &msg)
{
    if (msg.poses.size() <= kIndexTip) return;
    Hand &h = hands_[hand];

    const auto &thumb = msg.poses[kThumbTip].position;
    const auto &index = msg.poses[kIndexTip].position;
    const double gap  = std::sqrt(std::pow(thumb.x - index.x, 2) + std::pow(thumb.y - index.y, 2) +
                                 std::pow(thumb.z - index.z, 2));

    /* Open and close thresholds differ so a hand held near the boundary does not rattle between
     * grabbing and dropping. */
    if (!h.pinching && gap < conf_.pinch_close_m) h.pinching = true;
    else if (h.pinching && gap > conf_.pinch_open_m) h.pinching = false;

    /* Grab at the pinch point, which is where the object visually sits between the fingers,
     * but take orientation from the wrist: fingertip orientation is the noisiest thing the
     * tracker reports. */
    h.pos[0] = 0.5 * (thumb.x + index.x);
    h.pos[1] = 0.5 * (thumb.y + index.y);
    h.pos[2] = 0.5 * (thumb.z + index.z);
    const auto &wq = msg.poses[kWrist].orientation;
    h.quat[0]      = wq.w;
    h.quat[1]      = wq.x;
    h.quat[2]      = wq.y;
    h.quat[3]      = wq.z;
    h.have_pose    = true;
}

int Grabber::nearest_body(const mjData *data, const mjtNum point[3]) const
{
    int    best      = -1;
    double best_dist = conf_.reach_m;

    /* Body 0 is the world; it has infinite mass and grabbing it would do nothing. */
    for (int b = 1; b < model_->nbody; ++b) {
        if (model_->body_mass[b] <= 0.0) continue;
        const mjtNum *p = data->xipos + 3 * b; // centre of mass, which is what the force acts on
        const double  d = std::sqrt((p[0] - point[0]) * (p[0] - point[0]) +
                                    (p[1] - point[1]) * (p[1] - point[1]) +
                                    (p[2] - point[2]) * (p[2] - point[2]));
        if (d < best_dist) {
            best_dist = d;
            best      = b;
        }
    }
    return best;
}

void Grabber::apply(mjData *data)
{
    for (auto &[name, h] : hands_) {
        if (!h.have_pose) continue;

        const bool want = h.pressed || h.pinching;

        if (want && h.body < 0) {
            h.body = nearest_body(data, h.pos);
            if (h.body > 0) {
                /* Record where the body sits in the hand's frame, so it is dragged from where
                 * it was caught rather than snapping to the palm. */
                mjtNum hand_inv[4], rel[3];
                mju_negQuat(hand_inv, h.quat);
                mju_sub3(rel, data->xipos + 3 * h.body, h.pos);
                mju_rotVecQuat(h.grab_pos, rel, hand_inv);
                mju_mulQuat(h.grab_quat, hand_inv, data->xquat + 4 * h.body);
                RCLCPP_INFO(node_.get_logger(), "%s grabbed body %d (%s)", name.c_str(), h.body,
                            mj_id2name(model_, mjOBJ_BODY, h.body)
                              ? mj_id2name(model_, mjOBJ_BODY, h.body)
                              : "unnamed");
            }
        } else if (!want && h.body > 0) {
            mju_zero(data->xfrc_applied + 6 * h.body, 6);
            h.body = -1;
        }

        if (h.body <= 0) continue;

        // Target pose: the grab-time offset carried rigidly by the hand.
        mjtNum target_pos[3], target_quat[4], offset[3];
        mju_rotVecQuat(offset, h.grab_pos, h.quat);
        mju_add3(target_pos, h.pos, offset);
        mju_mulQuat(target_quat, h.quat, h.grab_quat);

        mjtNum vel[6]; // (rot, lin) in a body-centred frame with world orientation
        mj_objectVelocity(model_, data, mjOBJ_BODY, h.body, vel, 0);

        mjtNum force[3];
        mju_sub3(force, target_pos, data->xipos + 3 * h.body);
        mju_scl3(force, force, conf_.kp);
        mjtNum damping[3];
        mju_scl3(damping, vel + 3, conf_.kd);
        mju_subFrom3(force, damping);
        clamp3(force, conf_.max_force);

        /* Orientation error as a rotation vector: the axis-angle of target * current^-1. */
        mjtNum cur_inv[4], err_quat[4], err_rot[3];
        mju_negQuat(cur_inv, data->xquat + 4 * h.body);
        mju_mulQuat(err_quat, target_quat, cur_inv);
        mju_quat2Vel(err_rot, err_quat, 1.0);

        mjtNum torque[3];
        mju_scl3(torque, err_rot, conf_.kp_rot);
        mjtNum rot_damping[3];
        mju_scl3(rot_damping, vel, conf_.kd_rot);
        mju_subFrom3(torque, rot_damping);
        clamp3(torque, conf_.max_torque);

        mju_copy3(data->xfrc_applied + 6 * h.body, force);
        mju_copy3(data->xfrc_applied + 6 * h.body + 3, torque);
    }
}

int Grabber::held_body(const std::string &hand) const
{
    auto it = hands_.find(hand);
    return it == hands_.end() ? -1 : it->second.body;
}

} // namespace vr
