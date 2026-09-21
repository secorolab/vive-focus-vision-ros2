/* SPDX-License-Identifier: MIT */
#pragma once

#include <array>
#include <memory>
#include <string>
#include <mujoco/mujoco.h>

namespace vive_vr_ros2 {

/** Resolved-rate servo for the selected OpenArm arm, over the Orocos KDL chain.
 * It owns the commanded configuration, so one clutch press stays on one IK branch; the running
 * physics state is read-only, and `sync` is the only place measured joints enter. */
class OpenArmIk
{
  public:
    /** What the control law reads, from the node's parameters. */
    struct Gains
    {
        double joint_speed;    // [rad/s] bound on each commanded joint velocity
        double max_linear;     // [m/s] bound on commanded tool linear speed
        double max_angular;    // [rad/s] bound on commanded tool angular speed
        double tracking_tau;   // [s] time constant with which the tool closes its pose error
        double damping;        // least-squares damping at a singularity
        double posture_gain;   // [1/s] null-space pull towards mid-range
    };

    /** What the tick achieved, for status rather than for control. */
    struct Result
    {
        double position_error = 0;  // [m] left after this tick
        double rotation_error = 0;  // [rad] left after this tick
        bool   limited        = false; // a bound or the damping held the arm back
    };

    explicit OpenArmIk(mjModel *model, const std::string &arm = "right");
    ~OpenArmIk();
    OpenArmIk(const OpenArmIk &) = delete;
    OpenArmIk &operator=(const OpenArmIk &) = delete;

    /** Adopts the measured joints as the commanded ones. */
    void sync(const mjData *live);

    /** Advances the commanded configuration towards the target pose by one period. */
    Result step(const mjtNum *target_p, const mjtNum *target_q, double dt, const Gains &gains);

    /** Commanded joint positions, written to `motors`. */
    const std::array<double, 7> &command() const;

    int tcp;
    std::array<int, 7> joints, qpos, dofs, motors;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace vive_vr_ros2
