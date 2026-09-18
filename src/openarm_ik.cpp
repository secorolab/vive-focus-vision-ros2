/* SPDX-License-Identifier: MIT */
#include "vive_vr_ros2/openarm_ik.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <mj_kdl_wrapper/mj_kdl_wrapper.hpp>

namespace vive_vr_ros2 {

struct OpenArmIk::Impl
{
    mjModel *model;
    std::unique_ptr<mjData, decltype(&mj_deleteData)> scratch;
    mj_kdl::Robot robot;
    KDL::JntArray lower{7}, upper{7};
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk;
    std::unique_ptr<KDL::ChainIkSolverVel_pinv> velocity;
    std::unique_ptr<KDL::ChainIkSolverPos_NR_JL> bounded;
    std::unique_ptr<KDL::ChainIkSolverPos_LMA> lma;

    explicit Impl(mjModel *m) : model(m), scratch(mj_makeData(m), mj_deleteData) {}
};

OpenArmIk::OpenArmIk(mjModel *model) : impl_(std::make_unique<Impl>(model))
{
    auto &state = *impl_;
    tcp = mj_name2id(model, mjOBJ_BODY, "openarm_right_hand_tcp");
    if (tcp < 0 || !state.scratch ||
        !mj_kdl::init_robot_from_mjcf(&state.robot, model, state.scratch.get(),
                                    "world", "openarm_right_hand_tcp")) {
        throw std::runtime_error("Cannot build the OpenArm right TCP chain");
    }
    if (state.robot.n_joints != 7) {
        throw std::runtime_error("OpenArm right chain must have seven joints");
    }
    for (int i = 0; i < 7; ++i) {
        const std::string name = "openarm_right_joint" + std::to_string(i + 1);
        joints[i] = mj_name2id(model, mjOBJ_JOINT, name.c_str());
        qpos[i] = state.robot.kdl_to_mj_qpos[i];
        dofs[i] = state.robot.kdl_to_mj_dof[i];
        motors[i] = state.robot.kdl_to_mj_ctrl[i];
        if (joints[i] < 0 || motors[i] < 0 || state.robot.joint_names[i] != name ||
            model->jnt_type[joints[i]] != mjJNT_HINGE || !model->jnt_limited[joints[i]]) {
            throw std::runtime_error("Unexpected OpenArm joint/actuator: " + name);
        }
        state.lower(i) = model->jnt_range[2 * joints[i]];
        state.upper(i) = model->jnt_range[2 * joints[i] + 1];
    }
    state.fk = std::make_unique<KDL::ChainFkSolverPos_recursive>(state.robot.chain);
    state.velocity = std::make_unique<KDL::ChainIkSolverVel_pinv>(state.robot.chain);
    state.bounded = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(
      state.robot.chain, state.lower, state.upper, *state.fk, *state.velocity, 200, 1e-5);
    state.lma = std::make_unique<KDL::ChainIkSolverPos_LMA>(state.robot.chain, 1e-5, 200);
}

OpenArmIk::~OpenArmIk() = default;

bool OpenArmIk::solve(const mjData *live, const mjtNum *target_p, const mjtNum *target_q,
                     std::array<double, 7> &solution)
{
    for (int i = 0; i < 3; ++i) if (!std::isfinite(target_p[i])) return false;
    for (int i = 0; i < 4; ++i) if (!std::isfinite(target_q[i])) return false;
    auto &state = *impl_;
    const KDL::Frame target(
      KDL::Rotation::Quaternion(target_q[1], target_q[2], target_q[3], target_q[0]),
      KDL::Vector(target_p[0], target_p[1], target_p[2]));
    KDL::JntArray seed(7), result(7);

    // The current state is preferred. A bent seed handles the straight-down singularity;
    // both attempts are only numerical seeds, never changes to the simulated robot pose.
    for (int attempt = 0; attempt < 2; ++attempt) {
        for (int i = 0; i < 7; ++i) seed(i) = live->qpos[qpos[i]];
        if (attempt == 1) {
            seed(3) = std::max(0.3, seed(3));
            seed(6) = std::clamp(seed(6) - 0.3, state.lower(6), state.upper(6));
        }
        for (int solver = 0; solver < 2; ++solver) {
            const int status = solver == 0 ? state.bounded->CartToJnt(seed, target, result)
                                           : state.lma->CartToJnt(seed, target, result);
            if (status < 0) continue;
            bool within_limits = true;
            for (int i = 0; i < 7; ++i) {
                within_limits &= std::isfinite(result(i)) && result(i) >= state.lower(i) &&
                                 result(i) <= state.upper(i);
            }
            if (!within_limits) continue;

            // Validate against MuJoCo, independently of the KDL chain conversion and solver.
            mj_copyData(state.scratch.get(), state.model, live);
            for (int i = 0; i < 7; ++i) state.scratch->qpos[qpos[i]] = result(i);
            mj_kinematics(state.model, state.scratch.get());
            mjtNum position_error[3], inverse[4], rotation[4], rotation_error[3];
            mju_sub3(position_error, target_p, state.scratch->xpos + 3 * tcp);
            mju_negQuat(inverse, state.scratch->xquat + 4 * tcp);
            mju_mulQuat(rotation, target_q, inverse);
            if (rotation[0] < 0) for (auto &value : rotation) value = -value;
            mju_quat2Vel(rotation_error, rotation, 1.0);
            if (mju_norm3(position_error) > 0.003 || mju_norm3(rotation_error) > 0.04) continue;
            for (int i = 0; i < 7; ++i) solution[i] = result(i);
            return true;
        }
    }
    return false;
}

} // namespace vive_vr_ros2
