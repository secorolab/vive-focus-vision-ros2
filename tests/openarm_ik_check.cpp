/* Contract of the resolved-rate servo. Runs against tests/openarm_test_arm.xml by default, so it
 * needs no OpenArm description; pass a generated model to check the real robot too. */

#include "vive_vr_ros2/openarm_ik.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

constexpr double kDt = 1.0 / 60.0;

void check(bool condition, const std::string &message)
{
    if (!condition) throw std::runtime_error(message);
}

vive_vr_ros2::OpenArmIk::Gains gains()
{
    return { 1.5, 0.5, 2.0, 0.12, 0.05, 0.5 };
}

void reset(mjModel *m, mjData *d)
{
    if (m->nkey > 0) {
        mj_resetDataKeyframe(m, d, 0);
    } else {
        mj_resetData(m, d);
    }
    mj_forward(m, d);
}

/** Tool pose of the configuration `q`, taken from MuJoCo so it is independent of the KDL chain. */
void tool_of(mjModel *m, mjData *scratch, const vive_vr_ros2::OpenArmIk &ik,
             const std::array<double, 7> &q, mjtNum *p, mjtNum *quat)
{
    for (int i = 0; i < 7; ++i) scratch->qpos[ik.qpos[i]] = q[i];
    mj_kinematics(m, scratch);
    mju_copy3(p, scratch->xpos + 3 * ik.tcp);
    mju_copy4(quat, scratch->xquat + 4 * ik.tcp);
}

double rotation_between(const mjtNum *a, const mjtNum *b)
{
    mjtNum inverse[4], difference[4], vel[3];
    mju_negQuat(inverse, b);
    mju_mulQuat(difference, a, inverse);
    if (difference[0] < 0) for (auto &value : difference) value = -value;
    mju_quat2Vel(vel, difference, 1.0);
    return mju_norm3(vel);
}

/** Liegeois' joint-limit criterion: how far the arm sits from the middle of its ranges. */
double posture_cost(mjModel *m, const vive_vr_ros2::OpenArmIk &ik,
                    const std::array<double, 7> &q)
{
    double cost = 0;
    for (int i = 0; i < 7; ++i) {
        const double lower = m->jnt_range[2 * ik.joints[i]];
        const double upper = m->jnt_range[2 * ik.joints[i] + 1];
        const double normalised = (q[i] - 0.5 * (lower + upper)) / (0.5 * (upper - lower));
        cost += normalised * normalised;
    }
    return cost;
}

/** Runs the servo, asserting the per-tick bounds the control law promises on every tick. */
vive_vr_ros2::OpenArmIk::Result drive(vive_vr_ros2::OpenArmIk &ik, mjModel *m, const mjtNum *p,
                                      const mjtNum *quat, int ticks, const std::string &what)
{
    vive_vr_ros2::OpenArmIk::Result result;
    for (int tick = 0; tick < ticks; ++tick) {
        const std::array<double, 7> before = ik.command();
        result = ik.step(p, quat, kDt, gains());
        const std::array<double, 7> after = ik.command();
        for (int i = 0; i < 7; ++i) {
            check(std::isfinite(after[i]), what + ": joint " + std::to_string(i) + " is not finite");
            check(std::abs(after[i] - before[i]) <= gains().joint_speed * kDt + 1e-9,
                  what + ": joint " + std::to_string(i) + " moved faster than the speed bound");
            check(after[i] >= m->jnt_range[2 * ik.joints[i]] - 1e-9 &&
                  after[i] <= m->jnt_range[2 * ik.joints[i] + 1] + 1e-9,
                  what + ": joint " + std::to_string(i) + " left its range");
        }
    }
    return result;
}

} // namespace

int main(int argc, char **argv)
{
    const std::string path =
      argc == 2 ? argv[1] : std::string(TEST_SOURCE_DIR) + "/openarm_test_arm.xml";
    char error[2048] = {};
    mjModel *m = mj_loadXML(path.c_str(), nullptr, error, sizeof(error));
    if (!m) {
        std::cerr << "cannot load " << path << ": " << error << '\n';
        return 1;
    }
    mjData *d = mj_makeData(m);
    mjData *scratch = mj_makeData(m);
    int status = 0;
    try {
        vive_vr_ros2::OpenArmIk ik(m);
        reset(m, d);
        reset(m, scratch);

        // The commanded configuration starts from the measured one, and nothing else reads physics.
        ik.sync(d);
        for (int i = 0; i < 7; ++i) {
            check(std::abs(ik.command()[i] - d->qpos[ik.qpos[i]]) < 1e-9,
                  "sync did not adopt the measured joints");
        }
        const std::vector<double> untouched(d->qpos, d->qpos + m->nq);

        // A pose the arm can reach, described by a configuration but pursued as a tool pose.
        std::array<double, 7> elsewhere = ik.command();
        elsewhere[1] += 0.30;
        elsewhere[3] += 0.25;
        elsewhere[5] += 0.20;
        mjtNum target_p[3], target_q[4];
        tool_of(m, scratch, ik, elsewhere, target_p, target_q);

        auto reached = drive(ik, m, target_p, target_q, 300, "reachable");
        check(reached.position_error < 1e-3, "servo did not converge in position");
        check(reached.rotation_error < 1e-2, "servo did not converge in orientation");
        for (int i = 0; i < m->nq; ++i) {
            check(d->qpos[i] == untouched[i], "the servo wrote to the live simulation state");
        }

        // What it converged on must agree with MuJoCo, not just with its own KDL chain.
        mjtNum got_p[3], got_q[4];
        tool_of(m, scratch, ik, ik.command(), got_p, got_q);
        mjtNum gap[3];
        mju_sub3(gap, got_p, target_p);
        check(mju_norm3(gap) < 2e-3, "KDL and MuJoCo disagree about where the tool ended up");
        check(rotation_between(got_q, target_q) < 2e-2,
              "KDL and MuJoCo disagree about the tool orientation");

        // The commanded pose is one the actuators can actually hold.
        for (int i = 0; i < 7; ++i) d->ctrl[ik.motors[i]] = ik.command()[i];
        for (int step = 0; step < 2000; ++step) mj_step(m, d);
        mj_forward(m, d);
        mju_sub3(gap, d->xpos + 3 * ik.tcp, target_p);
        check(mju_norm3(gap) < 5e-3, "the actuators did not reach the commanded pose");

        // Spare freedom goes into posture: holding the tool still must still centre the joints.
        reset(m, d);
        ik.sync(d);
        std::array<double, 7> awkward = ik.command();
        awkward[0] = 0.9 * m->jnt_range[2 * ik.joints[0] + 1];
        tool_of(m, scratch, ik, awkward, target_p, target_q);
        drive(ik, m, target_p, target_q, 300, "posture approach");
        const double before_cost = posture_cost(m, ik, ik.command());
        const auto settled = drive(ik, m, target_p, target_q, 600, "posture hold");
        const double after_cost = posture_cost(m, ik, ik.command());
        check(after_cost < before_cost - 1e-4, "the null space did not move the arm towards mid-range");
        check(settled.position_error < 2e-3, "posture motion disturbed the tool position");
        check(settled.rotation_error < 2e-2, "posture motion disturbed the tool orientation");

        // Straight out, every Z axis aligned: the damping has to carry this without exploding.
        mj_resetData(m, d);
        mj_forward(m, d);
        ik.sync(d);
        mjtNum singular_p[3], singular_q[4];
        tool_of(m, scratch, ik, ik.command(), singular_p, singular_q);
        singular_p[0] += 0.05;
        drive(ik, m, singular_p, singular_q, 300, "singular start");

        // Out of reach must settle at the closest it can manage, not freeze and not diverge.
        reset(m, d);
        ik.sync(d);
        mjtNum here_p[3], far_q[4];
        tool_of(m, scratch, ik, ik.command(), here_p, far_q); // keep the orientation attainable
        const mjtNum far_p[3] = { 2.0, 2.0, 2.0 };
        const auto first = drive(ik, m, far_p, far_q, 50, "out of reach");
        const auto last = drive(ik, m, far_p, far_q, 400, "out of reach");
        check(last.position_error < first.position_error,
              "the arm did not extend towards an unreachable target");
        check(last.limited, "an unreachable target should report the arm as limited");

        std::printf("PASS: convergence, velocity and limit bounds, MuJoCo agreement, actuator "
                    "tracking, null-space posture, singularity, out-of-reach settling\n");
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        status = 1;
    }
    mj_deleteData(scratch);
    mj_deleteData(d);
    mj_deleteModel(m);
    return status;
}
