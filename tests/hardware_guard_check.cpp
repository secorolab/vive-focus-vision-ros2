#include "vive_vr_ros2/hardware_guard.hpp"
#include <limits>
#include <stdexcept>
#include <vector>
using namespace vive_vr_ros2;
void check(bool condition) { if (!condition) throw std::runtime_error("hardware guard check failed"); }
int main() {
    using Clock = std::chrono::steady_clock;
    Clock::time_point t{};
    std::vector<Clock::time_point> sends;
    auto now = [&] { return t; };
    auto wait = [&](Clock::time_point until) { t = until; };
    check(paced_motor_batch(now, wait, [&](int i) {
        check(i == static_cast<int>(sends.size()));
        sends.push_back(t);
        if (i == 2) t += std::chrono::milliseconds(2); // scheduling delay
        return true;
    }));
    check(sends.size() == 8);
    for (size_t i=1;i<sends.size();++i)
        check(sends[i]-sends[i-1] >= std::chrono::microseconds(250));
    // A single host delay seen on the physical system remains bounded and
    // must not compress subsequent frame spacing into a catch-up burst.
    sends.clear();
    bool delayed=false;
    check(paced_motor_batch(now, [&](Clock::time_point until) {
        t=until;
        if (!delayed) { t += std::chrono::milliseconds(24); delayed=true; }
    }, [&](int) { sends.push_back(t); return true; }));
    check(sends.size()==8);
    for (size_t i=1;i<sends.size();++i)
        check(sends[i]-sends[i-1] >= std::chrono::microseconds(250));
    int count=0;
    check(!paced_motor_batch(now, wait, [&](int i) { ++count; return i != 3; }));
    check(count==4); // Never retry failure or send the remaining motors.
    count=0;
    check(!paced_motor_batch(now, [&](Clock::time_point until) {
        t=until+std::chrono::milliseconds(31);
    }, [&](int) { ++count; return true; }));
    check(count==1); // A delayed wake must not issue the next stale command.
    check(valid_recovery_command(-0.01049,-0.01049,-0.01049,0,2.44,0.1)); // hold
    check(valid_recovery_command(-0.005,-0.01049,-0.01049,0,2.44,0.1)); // inward
    check(!valid_recovery_command(-0.015,-0.03,-0.01049,0,2.44,0.1)); // drift cannot extend
    check(!valid_recovery_command(-0.001,0,0,0,2.44,0.1)); // no re-entry
    check(!valid_recovery_command(2.45,2.44,2.44,0,2.44,0.1)); // upper limit
    check(bounded_measured_hold(-0.01049,-0.01049,0,0,2.44,0.001)==0);
    check(bounded_measured_hold(-0.015,-0.015,-0.01,0,2.44,0.001)==-0.01);
    check(bounded_measured_hold(-0.02,0,0,0,2.44,0.001)==-0.02); // unrelated bad target still rejected
    check(bounded_measured_hold(0.1,0,0,0,2.44,0.001)==0.1); // normal motion unchanged
    check(valid_recovery_command(-0.000136,-0.000136,-0.000136,0,0.044,0.01)); // measured gripper hold
    check(!valid_recovery_command(-0.00019,-0.000136,-0.000136,0,0.044,0.01)); // no further closing
    check(!valid_recovery_command(-0.0001,0,0,0,0.044,0.01)); // no negative goal after recovery
    check(!valid_recovery_command(0.0441,0.044,0.044,0,0.044,0.01)); // opening limit unchanged
    MotorObservation m{true,false,false,0.01,0.0,0.0};
    check(valid_motor(m));
    m.responded=false; check(!valid_motor(m)); m.responded=true;
    m.age_s=0.101; check(!valid_motor(m)); m.age_s=0.01;
    m.error=true; check(!valid_motor(m)); m.error=false;
    m.malformed=true; check(!valid_motor(m)); m.malformed=false;
    m.position=std::numeric_limits<double>::quiet_NaN(); check(!valid_motor(m));
    check(valid_command(0.02,0,-1,1,0.1));
    check(!valid_command(0.2,0,-1,1,0.1));
    check(!valid_command(1.1,1.05,-1,1,0.1));
    check(!valid_command(std::numeric_limits<double>::quiet_NaN(),0,-1,1,0.1));
    check(std::abs(limited_step(1,0,0.1,0.01)-0.001)<1e-12);
    check(std::abs(limited_step(-1,0,0.1,0.01)+0.001)<1e-12);
}
