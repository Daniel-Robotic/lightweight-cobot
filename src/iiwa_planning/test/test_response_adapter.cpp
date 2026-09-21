#include <cassert>
#include <iostream>
#include <moveit/planning_interface/planning_response_adapter.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/ruckig_traj_smoothing.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <pluginlib/class_loader.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

int main() {
  auto urdf = urdf::parseURDF(
      R"(<robot name="test"><link name="base"/><link name="tip"/><joint name="joint1" type="revolute"><parent link="base"/><child link="tip"/><axis xyz="0 0 1"/><limit lower="-2" upper="2" velocity="1" effort="1"/></joint></robot>)");
  auto srdf = std::make_shared<srdf::Model>();
  assert(srdf->initString(
      *urdf,
      R"(<robot name="test"><group name="arm"><joint name="joint1"/></group></robot>)"));
  auto model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
  auto bounds = model->getVariableBounds("joint1");
  bounds.acceleration_bounded_ = bounds.jerk_bounded_ = true;
  bounds.min_acceleration_ = -1;
  bounds.max_acceleration_ = 1;
  bounds.min_jerk_ = -2;
  bounds.max_jerk_ = 2;
  model->getJointModel("joint1")->setVariableBounds("joint1", bounds);
  auto trajectory =
      std::make_shared<robot_trajectory::RobotTrajectory>(model, "arm");
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  state.setVariableVelocity("joint1", 0);
  state.setVariableAcceleration("joint1", 0);
  trajectory->addSuffixWayPoint(state, 0);
  state.setVariablePosition("joint1", 1);
  trajectory->addSuffixWayPoint(state, .2);
  assert(trajectory_processing::TimeOptimalTrajectoryGeneration()
             .computeTimeStamps(*trajectory));
  pluginlib::ClassLoader<planning_interface::PlanningResponseAdapter> loader(
      "moveit_core", "planning_interface::PlanningResponseAdapter");
  auto guard = loader.createSharedInstance("iiwa_planning/LimitSplineDynamics");
  planning_interface::MotionPlanRequest req;
  req.group_name = "arm";
  req.max_velocity_scaling_factor = req.max_acceleration_scaling_factor = 1;
  planning_interface::MotionPlanResponse res;
  res.trajectory = trajectory;
  res.error_code = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  auto boundary = loader.createSharedInstance("iiwa_planning/SetRestBoundary");
  req.start_state.joint_state.name = {"joint1"};
  req.start_state.joint_state.position = {0};
  req.start_state.joint_state.velocity = {0};
  boundary->adapt(nullptr, req, res);
  assert(bool(res));
  assert(trajectory_processing::RuckigSmoothing::applySmoothing(*trajectory));
  boundary->adapt(nullptr, req, res);
  assert(bool(res));
  guard->adapt(nullptr, req, res);
  assert(bool(res));
  assert(trajectory->getDuration() > .2);
  state.setVariablePosition("joint1", 3);
  trajectory->addSuffixWayPoint(state, 1);
  guard->adapt(nullptr, req, res);
  assert(!bool(res));
  std::cout << "Installed Ruckig + pluginlib response adapter: success and "
               "rejection OK\n";
}
