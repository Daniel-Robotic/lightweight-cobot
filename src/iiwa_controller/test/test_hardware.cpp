#include <gtest/gtest.h>
#include "iiwa_controller/IIWAHardwareInterface.hpp"

using Hardware = iiwa_controller::IIWAHardwareInterface;
using Return = hardware_interface::CallbackReturn;

hardware_interface::HardwareComponentInterfaceParams parameters()
{
  hardware_interface::HardwareComponentInterfaceParams params;
  auto & info = params.hardware_info;
  info.name = "iiwa_test";
  info.type = "system";
  info.hardware_parameters["simulate"] = "true";
  for (int i = 1; i <= 7; ++i) {
    hardware_interface::ComponentInfo joint;
    joint.name = "joint" + std::to_string(i);
    for (const auto * name : {"position", "velocity", "effort"}) {
      hardware_interface::InterfaceInfo interface;
      interface.name = name;
      interface.data_type = "double";
      interface.initial_value = "0.2";
      joint.state_interfaces.push_back(interface);
    }
    hardware_interface::InterfaceInfo command;
    command.name = "position";
    command.data_type = "double";
    joint.command_interfaces.push_back(command);
    joint_limits::JointLimits limits;
    limits.has_position_limits = true;
    limits.min_position = -2.0;
    limits.max_position = 2.0;
    limits.has_velocity_limits = true;
    limits.max_velocity = 1.0;
    info.limits[joint.name] = limits;
    info.joints.push_back(joint);
  }
  return params;
}

TEST(Hardware, RejectsMalformedPortWithoutThrowing)
{
  Hardware hardware;
  auto params = parameters();
  params.hardware_info.hardware_parameters["fri_port"] = "30200oops";
  EXPECT_EQ(hardware.on_init(params), Return::ERROR);
}

TEST(Hardware, RejectsInvalidRealtimePriority)
{
  Hardware hardware;
  auto params = parameters();
  params.hardware_info.hardware_parameters["rt_prio"] = "0";
  EXPECT_EQ(hardware.on_init(params), Return::ERROR);
}

TEST(Hardware, RejectsInvalidPositionSmoothingTau)
{
  Hardware hardware;
  auto params = parameters();
  params.hardware_info.hardware_parameters["joint_position_tau"] = "-0.01";
  EXPECT_EQ(hardware.on_init(params), Return::ERROR);
}

TEST(Hardware, RejectsWrongJointOrderAndMissingInterfaces)
{
  auto params = parameters();
  std::swap(params.hardware_info.joints[0], params.hardware_info.joints[1]);
  Hardware wrong_order;
  EXPECT_EQ(wrong_order.on_init(params), Return::ERROR);
  params = parameters();
  params.hardware_info.joints[0].state_interfaces.pop_back();
  Hardware missing_interface;
  EXPECT_EQ(missing_interface.on_init(params), Return::ERROR);
}

TEST(Hardware, SimulationSeedsCommandsAndSupportsRepeatedLifecycle)
{
  Hardware hardware;
  ASSERT_EQ(hardware.on_init(parameters()), Return::SUCCESS);
  auto states = hardware.on_export_state_interfaces();
  auto commands = hardware.on_export_command_interfaces();
  const rclcpp_lifecycle::State state;
  ASSERT_EQ(hardware.on_configure(state), Return::SUCCESS);
  for (int cycle = 0; cycle < 2; ++cycle) {
    ASSERT_EQ(hardware.on_activate(state), Return::SUCCESS);
    const double command = hardware.get_command<double>("joint1/position");
    EXPECT_DOUBLE_EQ(command, 0.2);
    EXPECT_EQ(hardware.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
      hardware_interface::return_type::OK);
    EXPECT_EQ(hardware.on_deactivate(state), Return::SUCCESS);
    EXPECT_EQ(hardware.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
      hardware_interface::return_type::OK);
  }
  EXPECT_EQ(hardware.on_cleanup(state), Return::SUCCESS);
  EXPECT_EQ(hardware.on_shutdown(state), Return::SUCCESS);
}

TEST(Hardware, ConfiguredRealHardwareCleansUpInDependencyOrder)
{
  Hardware hardware;
  auto params = parameters();
  params.hardware_info.hardware_parameters["simulate"] = "false";
  params.hardware_info.hardware_parameters["robot_ip"] = "127.0.0.1";
  ASSERT_EQ(hardware.on_init(params), Return::SUCCESS);
  const rclcpp_lifecycle::State state;
  ASSERT_EQ(hardware.on_configure(state), Return::SUCCESS);
  EXPECT_EQ(hardware.on_cleanup(state), Return::SUCCESS);
  EXPECT_EQ(hardware.on_error(state), Return::SUCCESS);
}
