/* SPDX-License-Identifier: MIT */
#pragma once
#include <algorithm>
#include <cmath>
#include <chrono>
#include <string>

namespace vive_vr_ros2 {
// After a reset, observe release then a press before release may command closing.
inline int trigger_gripper_direction(bool pressed, bool &ready, int previous) {
    if (!pressed) {
        ready=true;
        return previous==0 ? 0 : -1;
    }
    return ready ? 1 : 0;
}
// Space actual sends, including after scheduler delays: never catch up in bursts.
// An incomplete batch faults; no retry or stale command queue is retained.
template<class Now, class Wait, class Send>
bool paced_motor_batch(Now now, Wait wait, Send send) {
    // Allow an occasional ~24 ms host scheduling delay while staying below
    // the independent 100 ms feedback/control-stall watchdogs. Never catch up.
    const auto deadline = now() + std::chrono::milliseconds(30);
    for (int i = 0; i < 8; ++i) {
        if (i) wait(now() + std::chrono::microseconds(250));
        if (now() >= deadline || !send(i)) return false;
    }
    return now() < deadline;
}
// Shared, deterministic checks used by the CAN driver and unit tests.
struct MotorObservation {
    bool responded, malformed, error;
    double age_s, position, velocity;
};
inline bool valid_motor(const MotorObservation &m) {
    return m.responded && !m.malformed && !m.error &&
           std::isfinite(m.age_s) && m.age_s >= 0 && m.age_s <= 0.1 &&
           std::isfinite(m.position) && std::isfinite(m.velocity);
}
inline bool valid_command(double desired, double measured, double lower, double upper,
                          double following_limit) {
    return std::isfinite(desired) && std::isfinite(measured) &&
           desired >= lower && desired <= upper &&
           std::abs(desired - measured) <= following_limit;
}
// An accepted startup target outside the nominal range may hold or move inward.
// Measured drift must never enlarge this corridor. Once inside, nominal limits apply.
inline bool valid_recovery_command(double desired, double measured, double previous,
                                   double lower, double upper, double following_limit) {
    return std::isfinite(previous) && valid_command(desired, measured,
        std::min(lower,previous), std::max(upper,previous), following_limit);
}
// JTC timeout holds measured position, which can sit just outside nominal zero.
// Map only a measured-pose hold back into the existing command corridor; never
// expand the corridor or accept an unrelated out-of-range target.
inline double bounded_measured_hold(double desired, double measured, double previous,
                                   double lower, double upper, double tolerance) {
    // JTC interpolates toward its measured timeout hold. If feedback sits
    // outside a boundary already recovered by the sent target, clamp that
    // entire interpolation segment to the boundary, never back outside.
    if (std::isfinite(desired) && std::isfinite(measured) && std::isfinite(previous)) {
        if (measured<lower && previous>=lower && desired>=measured && desired<lower) return lower;
        if (measured>upper && previous<=upper && desired<=measured && desired>upper) return upper;
    }
    if (std::isfinite(desired) && std::isfinite(measured) &&
        std::abs(desired-measured)<=tolerance)
        return std::clamp(desired,std::min(lower,previous),std::max(upper,previous));
    return desired;
}
inline double limited_step(double desired, double previous, double speed, double dt) {
    return previous + std::clamp(desired - previous, -speed * dt, speed * dt);
}
// Acceleration ramp and braking distance reduce start/stop steps. Clamp the
// integrated position at the endpoint; release/fault paths hold immediately.
inline double smooth_bounded_step(double goal, double previous, double &velocity,
                                  double speed, double acceleration, double dt) {
    const double distance=goal-previous;
    const double target=std::copysign(std::min(speed,std::sqrt(2*acceleration*std::abs(distance))),distance);
    velocity=limited_step(target,velocity,acceleration,dt);
    const double next=previous+velocity*dt;
    if ((distance>=0 && next>=goal) || (distance<=0 && next<=goal)) {
        velocity=0; return goal;
    }
    return next;
}

}
