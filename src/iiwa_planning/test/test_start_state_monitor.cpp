#include "iiwa_planning/start_state_monitor.hpp"
#include <cassert>
#include <iostream>
#include <thread>
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
      "iiwa_start_monitor_test", rclcpp::NodeOptions().enable_rosout(false));
  iiwa_planning::StartStateMonitor monitor;
  monitor.initialize(node);
  auto pub = node->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS());
  sensor_msgs::msg::JointState
      requested; // Normal MoveIt request has no velocities.
  assert(!monitor.stationary({"joint1"}, requested));
  sensor_msgs::msg::JointState state;
  state.name = {"joint1"};
  state.position = {0};
  state.velocity = {0};
  auto publish = [&] {
    for (int i = 0; i < 30; ++i) {
      state.header.stamp = node->now();
      pub->publish(state);
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  };
  publish();
  assert(monitor.stationary({"joint1"}, requested));
  state.velocity = {.1};
  publish();
  assert(!monitor.stationary({"joint1"}, requested));
  state.velocity = {0};
  publish();
  assert(monitor.stationary({"joint1"}, requested));
  std::this_thread::sleep_for(std::chrono::milliseconds(550));
  assert(!monitor.stationary({"joint1"}, requested));
  rclcpp::shutdown();
  std::cout << "Position-only MoveIt requests validated using fresh real "
               "velocity feedback\n";
}
