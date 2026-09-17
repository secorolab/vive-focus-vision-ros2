/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Vamsi Kalagaturu
 * See LICENSE for details. */

#include "vive_vr_ros2/grabber.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace vive_vr_ros2 {
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
            subs_.push_back(node.create_subscription<std_msgs::msg::Int32>(
              conf_.topic_ns + "/" + hand + "/target", rclcpp::SensorDataQoS(),
              [this, hand](std_msgs::msg::Int32::SharedPtr msg) { on_target(hand, *msg); }));
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

    /* The sender's own stamp, not the arrival time. Wi-Fi delivers these in bursts, so arrival
     * intervals collapse to near zero and a differenced velocity explodes. The client's clock
     * offset may be wrong, but it is constant between two consecutive poses, so it cancels here. */
    const double stamp_s = rclcpp::Time(msg.header.stamp).seconds();
    const double dt      = stamp_s - h.last_stamp_s;

    // Below kMinDt the difference is mostly quantisation noise; above kMaxDt the stream stalled.
    constexpr double kMinDt = 0.004;
    constexpr double kMaxDt = 0.25;

    if (h.have_pose && dt > kMinDt && dt < kMaxDt) {
        mjtNum lin[3], ang[3];
        mju_sub3(lin, h.pos, h.last_pos);
        mju_scl3(lin, lin, 1.0 / dt);

        mjtNum prev_inv[4], d_quat[4];
        mju_negQuat(prev_inv, h.last_quat);
        mju_mulQuat(d_quat, h.quat, prev_inv);
        if (d_quat[0] < 0) mju_scl(d_quat, d_quat, -1.0, 4);
        mju_quat2Vel(ang, d_quat, dt);

        // No hand moves this fast; anything above it is a jitter artefact, not motion.
        clamp3(lin, conf_.max_hand_speed);
        clamp3(ang, conf_.max_hand_turn_rate);

        /* One differenced sample per message is still noisy, and the damper multiplies it by kd.
         * A light low-pass costs a little feedforward accuracy and buys a lot of steadiness. */
        const double a = conf_.vel_filter;
        for (int i = 0; i < 3; ++i) {
            h.lin_vel[i] = a * lin[i] + (1.0 - a) * h.lin_vel[i];
            h.ang_vel[i] = a * ang[i] + (1.0 - a) * h.ang_vel[i];
        }
        h.have_vel = true;
    } else if (dt >= kMaxDt || dt < 0.0) {
        // A long gap, or a clock that jumped backwards: a difference across it is meaningless.
        mju_zero3(h.lin_vel);
        mju_zero3(h.ang_vel);
        h.have_vel = false;
    }

    mju_copy3(h.last_pos, h.pos);
    mju_copy(h.last_quat, h.quat, 4);
    h.last_stamp_s = stamp_s;
    h.have_pose    = true;
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

void Grabber::on_target(const std::string &hand, const std_msgs::msg::Int32 &msg)
{
    const int b = msg.data;
    // Body 0 is the world, and an index past the model would be a stale manifest on the client.
    hands_[hand].target = (b > 0 && b < model_->nbody) ? b : -1;
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
            // What the user is pointing at beats what happens to be nearest the controller.
            h.body = h.target > 0 ? h.target : nearest_body(data, h.pos);
            // nearest_body already skips these; a pointed-at body has not been checked.
            if (h.body > 0 && model_->body_mass[h.body] <= 0.0) h.body = -1;
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

        /* How fast the target itself is moving, from the hand's velocity carried rigidly through
         * the grab offset: v = v_hand + omega x offset. Without this the damper fights the body's
         * absolute velocity, so following a hand at a steady speed leaves a standing error of
         * kd*v/kp - the object visibly trails behind the ray. */
        mjtNum target_vel[3] = { 0, 0, 0 };
        const mjtNum *target_angvel = h.ang_vel;
        if (h.have_vel) {
            mju_cross(target_vel, h.ang_vel, offset);
            mju_addTo3(target_vel, h.lin_vel);
        }

        mjtNum vel[6]; // (rot, lin) in a body-centred frame with world orientation
        mj_objectVelocity(model_, data, mjOBJ_BODY, h.body, vel, 0);

        /* Gains are accelerations, scaled here by the body's own mass and inertia. A force-
         * limited spring behaves completely differently on a 0.15 kg ball and a 2 kg link -
         * the same clamp is 1300 m/s^2 for one and 100 m/s^2 for the other - so a reach that
         * merely tugs a robot arm launches a loose object across the world. */
        const mjtNum mass = model_->body_mass[h.body];
        // Diagonal of the inertia tensor in the inertial frame; the mean is close enough here.
        const mjtNum *diag = model_->body_inertia + 3 * h.body;
        const mjtNum inertia = (diag[0] + diag[1] + diag[2]) / 3.0;

        mjtNum accel[3];
        mju_sub3(accel, target_pos, data->xipos + 3 * h.body);
        mju_scl3(accel, accel, conf_.kp);
        mjtNum rel_vel[3], damping[3];
        mju_sub3(rel_vel, vel + 3, target_vel);
        mju_scl3(damping, rel_vel, conf_.kd);
        mju_subFrom3(accel, damping);
        clamp3(accel, conf_.max_accel);

        mjtNum force[3];
        mju_scl3(force, accel, mass);

        /* Orientation error as a rotation vector: the axis-angle of target * current^-1. */
        mjtNum cur_inv[4], err_quat[4], err_rot[3];
        mju_negQuat(cur_inv, data->xquat + 4 * h.body);
        mju_mulQuat(err_quat, target_quat, cur_inv);

        /* q and -q are the same orientation, but quat2Vel reads the negative-w one as a turn of
         * more than half a circle. Left alone the body chases the long way round, flips sign
         * again on the way, and spins on its axis without ever arriving. */
        if (err_quat[0] < 0) mju_scl(err_quat, err_quat, -1.0, 4);
        mju_quat2Vel(err_rot, err_quat, 1.0);

        mjtNum ang_accel[3];
        mju_scl3(ang_accel, err_rot, conf_.kp_rot);
        mjtNum rel_angvel[3], rot_damping[3];
        mju_sub3(rel_angvel, vel, target_angvel);
        mju_scl3(rot_damping, rel_angvel, conf_.kd_rot);
        mju_subFrom3(ang_accel, rot_damping);
        clamp3(ang_accel, conf_.max_ang_accel);

        mjtNum torque[3];
        mju_scl3(torque, ang_accel, inertia);

        mju_copy3(data->xfrc_applied + 6 * h.body, force);
        mju_copy3(data->xfrc_applied + 6 * h.body + 3, torque);
    }
}

int Grabber::held_body(const std::string &hand) const
{
    auto it = hands_.find(hand);
    return it == hands_.end() ? -1 : it->second.body;
}

} // namespace vive_vr_ros2
