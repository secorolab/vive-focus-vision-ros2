/* SPDX-License-Identifier: MIT */
#include "vive_vr_ros2/openarm_ik.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <mj_kdl_wrapper/mj_kdl_wrapper.hpp>

namespace vive_vr_ros2 {
namespace {

/* Smallest singular value at which damping starts being phased in; a length, in m/rad. */
constexpr double kSingularThreshold = 0.05;

/* Short of the mechanical stop, so the position servo cannot wind up against one. */
constexpr double kLimitMargin = 0.01;

} // namespace

struct OpenArmIk::Impl
{
    mjModel *model;
    std::unique_ptr<mjData, decltype(&mj_deleteData)> scratch;
    mj_kdl::Robot robot;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk;
    std::unique_ptr<KDL::ChainJntToJacSolver> jacobian;
    std::array<double, 7> lower{}, upper{}, middle{}, half_range{};
    std::array<double, 7> command{};

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
        state.lower[i] = model->jnt_range[2 * joints[i]] + kLimitMargin;
        state.upper[i] = model->jnt_range[2 * joints[i] + 1] - kLimitMargin;
        if (state.lower[i] >= state.upper[i]) {
            throw std::runtime_error("OpenArm joint range is too narrow to command: " + name);
        }
        state.middle[i] = 0.5 * (state.lower[i] + state.upper[i]);
        state.half_range[i] = 0.5 * (state.upper[i] - state.lower[i]);
    }
    state.fk = std::make_unique<KDL::ChainFkSolverPos_recursive>(state.robot.chain);
    state.jacobian = std::make_unique<KDL::ChainJntToJacSolver>(state.robot.chain);
}

OpenArmIk::~OpenArmIk() = default;

const std::array<double, 7> &OpenArmIk::command() const { return impl_->command; }

void OpenArmIk::sync(const mjData *live)
{
    auto &state = *impl_;
    for (int i = 0; i < 7; ++i) {
        state.command[i] = std::clamp(live->qpos[qpos[i]], state.lower[i], state.upper[i]);
    }
}

OpenArmIk::Result OpenArmIk::step(const mjtNum *target_p, const mjtNum *target_q, double dt,
                                  const Gains &gains)
{
    auto &state = *impl_;
    Result out;
    for (int i = 0; i < 3; ++i) if (!std::isfinite(target_p[i])) return out;
    for (int i = 0; i < 4; ++i) if (!std::isfinite(target_q[i])) return out;
    if (!std::isfinite(dt) || dt <= 0) return out;

    KDL::JntArray q(7);
    for (int i = 0; i < 7; ++i) q(i) = state.command[i];

    KDL::Frame current;
    if (state.fk->JntToCart(q, current) < 0) return out;

    const KDL::Frame target(
      KDL::Rotation::Quaternion(target_q[1], target_q[2], target_q[3], target_q[0]),
      KDL::Vector(target_p[0], target_p[1], target_p[2]));

    /* Pose error as one twist: translation, and the rotation vector taking current to target. */
    const KDL::Twist error = KDL::diff(current, target);

    Eigen::Matrix<double, 6, 1> twist;
    for (int i = 0; i < 3; ++i) {
        twist(i) = error.vel[i] / gains.tracking_tau;
        twist(i + 3) = error.rot[i] / gains.tracking_tau;
    }

    /* Each half of the twist is scaled as a whole, so bounding the speed never bends the path. */
    const double linear = twist.head<3>().norm();
    const double angular = twist.tail<3>().norm();
    if (linear > gains.max_linear) {
        twist.head<3>() *= gains.max_linear / linear;
        out.limited = true;
    }
    if (angular > gains.max_angular) {
        twist.tail<3>() *= gains.max_angular / angular;
        out.limited = true;
    }

    KDL::Jacobian jac(7);
    if (state.jacobian->JntToJac(q, jac) < 0) return out;
    const Eigen::Matrix<double, 6, 7> J = jac.data;

    /* Damped least squares, damping raised only as a singularity is approached, so tracking
     * stays exact elsewhere (Nakamura and Hanafusa 1986; Chiaverini, Oriolo and Walker 1994). */
    const Eigen::JacobiSVD<Eigen::Matrix<double, 6, 7>> svd(
      J, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto &sigma = svd.singularValues();
    const double smallest = sigma(sigma.size() - 1);
    double lambda = 0.0;
    if (smallest < kSingularThreshold) {
        const double ratio = smallest / kSingularThreshold;
        lambda = gains.damping * std::sqrt(1.0 - ratio * ratio);
        out.limited = true;
    }

    Eigen::Matrix<double, 7, 6> pseudo = Eigen::Matrix<double, 7, 6>::Zero();
    for (Eigen::Index i = 0; i < sigma.size(); ++i) {
        const double s = sigma(i);
        const double factor = s / (s * s + lambda * lambda);
        pseudo += factor * svd.matrixV().col(i) * svd.matrixU().col(i).transpose();
    }
    Eigen::Matrix<double, 7, 1> qdot = pseudo * twist;

    /* The seventh joint is free once the tool pose is fixed: spend it drifting towards mid-range,
     * which is the classic joint-limit criterion projected into the null space (Liegeois 1977). */
    Eigen::Matrix<double, 7, 1> posture;
    for (int i = 0; i < 7; ++i) {
        posture(i) = gains.posture_gain * (state.middle[i] - q(i)) / state.half_range[i];
    }
    qdot += (Eigen::Matrix<double, 7, 7>::Identity() - pseudo * J) * posture;

    if (!qdot.allFinite()) return out;

    /* One scale for all seven, because per-joint clipping would change the tool's direction. */
    const double fastest = qdot.cwiseAbs().maxCoeff();
    if (fastest > gains.joint_speed) {
        qdot *= gains.joint_speed / fastest;
        out.limited = true;
    }

    for (int i = 0; i < 7; ++i) {
        state.command[i] =
          std::clamp(state.command[i] + qdot(i) * dt, state.lower[i], state.upper[i]);
        q(i) = state.command[i];
    }

    KDL::Frame reached;
    if (state.fk->JntToCart(q, reached) >= 0) {
        const KDL::Twist remaining = KDL::diff(reached, target);
        out.position_error = remaining.vel.Norm();
        out.rotation_error = remaining.rot.Norm();
    }
    return out;
}

} // namespace vive_vr_ros2
