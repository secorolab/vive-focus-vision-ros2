/* SPDX-License-Identifier: MIT */
#pragma once

#include <array>
#include <memory>
#include <mujoco/mujoco.h>

namespace vive_vr_ros2 {

/** OpenArm adapter around the existing Orocos KDL solvers, with MuJoCo FK validation.
 * The running physics state is read-only; solutions are actuator targets, not qpos writes. */
class OpenArmIk
{
  public:
    explicit OpenArmIk(mjModel *model);
    ~OpenArmIk();
    OpenArmIk(const OpenArmIk &) = delete;
    OpenArmIk &operator=(const OpenArmIk &) = delete;

    bool solve(const mjData *live, const mjtNum *position, const mjtNum *quaternion,
               std::array<double, 7> &solution);

    int tcp;
    std::array<int, 7> joints, qpos, dofs, motors;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace vive_vr_ros2
