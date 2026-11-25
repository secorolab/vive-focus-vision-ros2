/* SPDX-License-Identifier: MIT */
#pragma once
#include <array>
#include <functional>
#include <string>
#include <rclcpp/rclcpp.hpp>
namespace vive_vr_ros2 {
struct ExternalControlSnapshot {
    std::array<std::string,8> names;
    std::array<double,8> measured{}, lower{}, upper{};
};
struct ExternalControlHooks {
    std::function<std::string()> blockers;
    std::function<ExternalControlSnapshot()> snapshot;
    std::function<bool()> healthy;
    std::function<void(const std::array<double,8>&)> send;
    std::function<void()> hold;
    std::function<void(const char*)> stop;
};
class ExternalControl {
public:
    virtual ~ExternalControl()=default;
    virtual bool active() const=0;
    virtual void tick(double dt)=0;
    virtual void stop(const char *reason)=0;
};
using CreateExternalControl=ExternalControl* (*)(rclcpp::Node*,ExternalControlHooks);
}
