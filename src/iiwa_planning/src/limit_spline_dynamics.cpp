#include "iiwa_planning/spline_limits.hpp"
#include "iiwa_planning/start_state_monitor.hpp"
#include <moveit/planning_interface/planning_response_adapter.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace iiwa_planning {
// TOTG can supply nonzero endpoint acceleration. Establish rest boundary
// conditions before Ruckig, which otherwise preserves them. Positions and
// interior derivatives stay intact.
class SetRestBoundary : public planning_interface::PlanningResponseAdapter {
public:
  void initialize(const rclcpp::Node::SharedPtr &node,
                  const std::string &) override {
    start_monitor_.initialize(node);
  }
  std::string getDescription() const override {
    return "Establish intended rest boundaries around Ruckig";
  }
  void adapt(const planning_scene::PlanningSceneConstPtr &,
             const planning_interface::MotionPlanRequest &req,
             planning_interface::MotionPlanResponse &res) const override {
    if (!res.trajectory || res.trajectory->getWayPointCount() < 2 ||
        !res.trajectory->getGroup()) {
      res.error_code = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
      return;
    }
    if (!start_monitor_.stationary(
            res.trajectory->getGroup()->getVariableNames(),
            req.start_state.joint_state)) {
      RCLCPP_ERROR(
          rclcpp::get_logger("iiwa_spline_limits"),
          "Fresh, stationary joint_states required for rest-to-rest planning");
      res.error_code = moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE;
      return;
    }
    for (const auto index :
         {size_t(0), res.trajectory->getWayPointCount() - 1}) {
      auto &state = *res.trajectory->getWayPointPtr(index);
      for (const auto &name : res.trajectory->getGroup()->getVariableNames()) {
        state.setVariableVelocity(name, 0.0);
        state.setVariableAcceleration(name, 0.0);
      }
    }
  }

private:
  StartStateMonitor start_monitor_;
};
class LimitSplineDynamics : public planning_interface::PlanningResponseAdapter {
public:
  void initialize(const rclcpp::Node::SharedPtr &node,
                  const std::string &) override {
    start_monitor_.initialize(node);
  }
  std::string getDescription() const override {
    return "Limit JTC quintic velocity, acceleration and jerk";
  }
  void adapt(const planning_scene::PlanningSceneConstPtr &,
             const planning_interface::MotionPlanRequest &req,
             planning_interface::MotionPlanResponse &res) const override {
    if (!res.trajectory || res.trajectory->getWayPointCount() < 2) {
      res.error_code = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
      return;
    }
    try {
      auto &trajectory = *res.trajectory;
      const auto *group = trajectory.getGroup();
      if (!group)
        throw std::runtime_error("Missing trajectory group");
      const auto &names = group->getVariableNames();
      if (!start_monitor_.stationary(names, req.start_state.joint_state))
        throw std::runtime_error(
            "Fresh stationary joint_states required before spline validation");
      const auto model = trajectory.getRobotModel();
      auto scaling = [](double x) {
        return std::isfinite(x) && x > 0 && x <= 1 ? x : 1.0;
      };
      std::vector<Limit> limits;
      for (const auto &name : names) {
        const auto &b = model->getVariableBounds(name);
        if (!b.position_bounded_ || !b.velocity_bounded_ ||
            !b.acceleration_bounded_ || !b.jerk_bounded_)
          throw std::runtime_error(
              "Missing position/velocity/acceleration/jerk bounds: " + name);
        limits.push_back(
            {b.min_position_, b.max_position_,
             b.max_velocity_ * scaling(req.max_velocity_scaling_factor),
             b.max_acceleration_ * scaling(req.max_acceleration_scaling_factor),
             b.max_jerk_});
      }
      std::vector<Sample> samples;
      double t = 0;
      for (size_t i = 0; i < trajectory.getWayPointCount(); ++i) {
        const auto &state = *trajectory.getWayPointPtr(i);
        if (!state.hasVelocities() || !state.hasAccelerations())
          throw std::runtime_error(
              "JTC guard requires positions, velocities and accelerations");
        t += trajectory.getWayPointDurationFromPrevious(i);
        Sample s{t, {}, {}, {}};
        for (const auto &name : names) {
          s.position.push_back(state.getVariablePosition(name));
          s.velocity.push_back(state.getVariableVelocity(name));
          s.acceleration.push_back(state.getVariableAcceleration(name));
        }
        samples.push_back(s);
      }
      const double scale = limit_splines(samples, limits);
      if (scale > 1) {
        for (size_t i = 0; i < samples.size(); ++i) {
          auto &state = *trajectory.getWayPointPtr(i);
          for (size_t j = 0; j < names.size(); ++j) {
            state.setVariableVelocity(names[j], samples[i].velocity[j]);
            state.setVariableAcceleration(names[j], samples[i].acceleration[j]);
          }
          trajectory.setWayPointDurationFromPrevious(
              i, trajectory.getWayPointDurationFromPrevious(i) * scale);
        }
        RCLCPP_INFO(
            rclcpp::get_logger("iiwa_spline_limits"),
            "Trajectory duration scaled by %.6f to bound JTC derivatives",
            scale);
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(rclcpp::get_logger("iiwa_spline_limits"),
                   "Rejected trajectory: %s", e.what());
      res.error_code = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
    }
  }

private:
  StartStateMonitor start_monitor_;
};
} // namespace iiwa_planning
PLUGINLIB_EXPORT_CLASS(iiwa_planning::LimitSplineDynamics,
                       planning_interface::PlanningResponseAdapter)

PLUGINLIB_EXPORT_CLASS(iiwa_planning::SetRestBoundary,
                       planning_interface::PlanningResponseAdapter)
