#pragma once
#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>

namespace iiwa_planning {
// MoveIt's CurrentStateMonitor may omit velocities (copy_dynamics=false).
// Observe the original telemetry instead of treating missing dynamics as zero.
class StartStateMonitor {
public:
  void initialize(const rclcpp::Node::SharedPtr &node) {
    node_ = node;
    for (const auto &item : std::vector<std::pair<std::string, double>>{
             {"iiwa_dynamics.stopped_velocity_tolerance", .01},
             {"iiwa_dynamics.state_timeout", .5}})
      if (!node->has_parameter(item.first))
        node->declare_parameter(item.first, item.second);
    tolerance_ = node->get_parameter("iiwa_dynamics.stopped_velocity_tolerance")
                     .as_double();
    timeout_ = node->get_parameter("iiwa_dynamics.state_timeout").as_double();
    if (!std::isfinite(tolerance_) || tolerance_ <= 0 ||
        !std::isfinite(timeout_) || timeout_ <= 0)
      throw std::runtime_error("Invalid start-state monitoring parameters");
    subscription_ = node->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_ = msg;
          received_ = std::chrono::steady_clock::now();
        });
  }
  bool stationary(const std::vector<std::string> &names,
                  const sensor_msgs::msg::JointState &requested) const {
    // Explicit nonzero requested initial dynamics are never silently discarded.
    if (!requested.velocity.empty() && !velocities_ok(names, requested))
      return false;
    auto node = node_.lock();
    if (!node)
      return velocities_ok(names,
                           requested); // offline use requires explicit dynamics
    std::lock_guard<std::mutex> lock(mutex_);
    if (!latest_ || std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - received_)
                            .count() > timeout_)
      return false;
    const double age =
        node->now().seconds() -
        (latest_->header.stamp.sec + latest_->header.stamp.nanosec * 1e-9);
    return std::isfinite(age) && age >= 0 && age <= timeout_ &&
           velocities_ok(names, *latest_);
  }

private:
  bool velocities_ok(const std::vector<std::string> &names,
                     const sensor_msgs::msg::JointState &msg) const {
    for (const auto &name : names) {
      auto found = std::find(msg.name.begin(), msg.name.end(), name);
      if (found == msg.name.end())
        return false;
      const size_t i = found - msg.name.begin();
      if (i >= msg.velocity.size() || !std::isfinite(msg.velocity[i]) ||
          std::abs(msg.velocity[i]) > tolerance_)
        return false;
    }
    return true;
  }
  std::weak_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  mutable std::mutex mutex_;
  sensor_msgs::msg::JointState::ConstSharedPtr latest_;
  std::chrono::steady_clock::time_point received_;
  double tolerance_{.01}, timeout_{.5};
};
} // namespace iiwa_planning
