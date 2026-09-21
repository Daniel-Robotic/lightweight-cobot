#include "iiwa_planning/spline_limits.hpp"
#include <cassert>
#include <iostream>
#include <joint_trajectory_controller/trajectory.hpp>

int main() {
  using namespace iiwa_planning;
  std::vector<Sample> samples{{0, {0}, {0}, {0}}, {.2, {1}, {0}, {0}}};
  std::vector<Limit> limits{{-2, 2, 1, 1, 2}};
  const auto scale = limit_splines(samples, limits);
  assert(scale > 1);
  joint_trajectory_controller::Trajectory jtc;
  trajectory_msgs::msg::JointTrajectoryPoint a, b, out;
  a.positions = samples[0].position;
  a.velocities = samples[0].velocity;
  a.accelerations = samples[0].acceleration;
  b.positions = samples[1].position;
  b.velocities = samples[1].velocity;
  b.accelerations = samples[1].acceleration;
  const auto finish = rclcpp::Time(static_cast<int64_t>(samples[1].time * 1e9));
  const auto start = rclcpp::Time(int64_t(0));
  double prev = 0;
  const int n = 10000;
  for (int i = 0; i <= n; ++i) {
    const auto now = rclcpp::Time(finish.nanoseconds() * i / n);
    jtc.interpolate_between_points(start, a, finish, b, now, out);
    assert(std::abs(out.velocities[0]) <= 1.000001);
    assert(std::abs(out.accelerations[0]) <= 1.000001);
    if (i)
      assert(std::abs((out.accelerations[0] - prev) / (samples[1].time / n)) <=
             2.00001);
    prev = out.accelerations[0];
  }
  std::cout << "Installed JTC interpolation stays within "
               "velocity/acceleration/jerk limits\n";
}
