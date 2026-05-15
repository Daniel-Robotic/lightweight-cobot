#pragma once

#include <array>
#include <cmath>
#include <limits>
#include <string>

#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/rclcpp.hpp"

#include "iiwa_controller_v2/fri_client.hpp"

namespace iiwa_controller_v2
{

// Checks commands against joint limits parsed from the URDF before they reach the FRI client.
// Limits are read from <command_interface name="position" min="..." max="..."/> and
// <command_interface name="effort" max="..."/> in the robot description.
struct CommandGuard
{
  struct JointLimits
  {
    std::string name;
    double min_position{-std::numeric_limits<double>::infinity()};
    double max_position{std::numeric_limits<double>::infinity()};
    double max_torque{std::numeric_limits<double>::infinity()};
  };

  std::array<JointLimits, FRIClient::N_JOINTS> limits;

  // Populate limits from URDF hardware info. Call once in on_init.
  void configure(const hardware_interface::HardwareInfo & info)
  {
    for (std::size_t i = 0; i < FRIClient::N_JOINTS; ++i) {
      const auto & joint = info.joints[i];
      limits[i].name = joint.name;

      for (const auto & ci : joint.command_interfaces) {
        if (ci.name == "position") {
          if (!ci.min.empty()) {
            limits[i].min_position = std::stod(ci.min);
          }
          if (!ci.max.empty()) {
            limits[i].max_position = std::stod(ci.max);
          }
        } else if (ci.name == "effort") {
          if (!ci.max.empty()) {
            limits[i].max_torque = std::stod(ci.max);
          }
        }
      }
    }
  }

  // Returns false and logs the violation if any position command is out of range.
  bool check_position(const std::array<double, FRIClient::N_JOINTS> & pos,
                      const char * logger_name) const
  {
    for (std::size_t i = 0; i < FRIClient::N_JOINTS; ++i) {
      if (!std::isfinite(pos[i])) {
        continue;  // NaN/inf filtering is already done in FRIClient
      }
      if (pos[i] < limits[i].min_position || pos[i] > limits[i].max_position) {
        RCLCPP_ERROR(
          rclcpp::get_logger(logger_name),
          "Position command for '%s' = %.4f rad is outside limits [%.4f, %.4f]",
          limits[i].name.c_str(), pos[i],
          limits[i].min_position, limits[i].max_position);
        return false;
      }
    }
    return true;
  }

  // Returns false and logs the violation if any torque command exceeds the limit.
  bool check_torque(const std::array<double, FRIClient::N_JOINTS> & tau,
                    const char * logger_name) const
  {
    for (std::size_t i = 0; i < FRIClient::N_JOINTS; ++i) {
      if (!std::isfinite(tau[i])) {
        continue;
      }
      if (std::abs(tau[i]) > limits[i].max_torque) {
        RCLCPP_ERROR(
          rclcpp::get_logger(logger_name),
          "Torque command for '%s' = %.2f Nm exceeds limit %.2f Nm",
          limits[i].name.c_str(), tau[i], limits[i].max_torque);
        return false;
      }
    }
    return true;
  }
};

}  // namespace iiwa_controller_v2
