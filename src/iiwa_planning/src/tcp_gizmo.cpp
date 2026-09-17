#include <iiwa_planning/tcp_gizmo_policy.hpp>
#include <iiwa_msgs/msg/gizmo_command.hpp>
#include <iiwa_msgs/msg/gizmo_state.hpp>
#include <iiwa_msgs/srv/motion_priority.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_servo/servo.hpp>
#include <moveit_servo/utils/common.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <deque>
#include <functional>
#include <map>
#include <stdexcept>

namespace iiwa_planning {
class TcpGizmo {
  using Trajectory = trajectory_msgs::msg::JointTrajectory;
  using Priority = iiwa_msgs::srv::MotionPriority;
  using Follow = control_msgs::action::FollowJointTrajectory;
  using ClientGoal = rclcpp_action::ClientGoalHandle<Follow>;
  using ServerGoal = rclcpp_action::ServerGoalHandle<Follow>;
  struct ProxyGoal {
    std::shared_ptr<ServerGoal> upstream;
    ClientGoal::SharedPtr downstream;
    bool cancel_requested{false};
    bool cancel_sent{false};
    bool finished{false};
  };
public:
  explicit TcpGizmo(const rclcpp::Node::SharedPtr& node) : node_(node) {
    if (!param<bool>("enabled", false) || !param<bool>("simulation", false))
      throw std::runtime_error("TCP gizmo requires enabled=true and simulation=true");
    group_name_ = param<std::string>("planning_group", "iiwa_arm");
    tcp_ = param<std::string>("pose_link", "tcp");
    frame_ = param<std::string>("default_frame", "base_link");
    period_ = positive("publish_period", .032);
    command_timeout_ = positive("command_timeout", .3);
    state_timeout_ = positive("state_timeout", .5);
    linear_speed_ = positive("max_linear_speed", .10);
    angular_speed_ = positive("max_angular_speed", .4);
    listener_ = std::make_shared<servo::ParamListener>(node_, "moveit_servo");
    const auto params = listener_->get_params();
    if (params.move_group_name != group_name_ || std::abs(params.publish_period - period_) > 1e-9)
      throw std::runtime_error("Servo group/period must match TCP gizmo parameters");
    scene_ = moveit_servo::createPlanningSceneMonitor(node_, params);
    if (!scene_ || !scene_->getRobotModel()) throw std::runtime_error("Missing planning scene");
    group_ = scene_->getRobotModel()->getJointModelGroup(group_name_);
    if (!group_ || !group_->getSolverInstance()) throw std::runtime_error("Missing IK solver");
    tip_ = group_->getSolverInstance()->getTipFrame();
    const auto model = scene_->getRobotModel();
    if (!model->hasLinkModel(tcp_) || !model->hasLinkModel(frame_) || !model->hasLinkModel(tip_))
      throw std::runtime_error("Unknown TCP, frame or solver tip");
    if (moveit::core::RobotModel::getRigidlyConnectedParentLinkModel(model->getLinkModel(tcp_)) !=
        moveit::core::RobotModel::getRigidlyConnectedParentLinkModel(model->getLinkModel(tip_)))
      throw std::runtime_error("TCP must be rigidly attached to the IK solver tip");
    output_ = node_->create_publisher<Trajectory>("/cobot/tcp_gizmo/controller_command", 10);
    state_pub_ = node_->create_publisher<iiwa_msgs::msg::GizmoState>("/cobot/tcp_gizmo/state", 1);
    command_sub_ = node_->create_subscription<iiwa_msgs::msg::GizmoCommand>(
      "/cobot/tcp_gizmo/command", 1, [this](const iiwa_msgs::msg::GizmoCommand& msg) {
        try { command(msg); }
        catch (const std::exception& e) { reject(std::string("target error: ") + e.what()); }
      });
    action_client_ = rclcpp_action::create_client<Follow>(node_, "/cobot/tcp_gizmo/follow_joint_trajectory");
    action_server_ = rclcpp_action::create_server<Follow>(node_, "/iiwa_arm_controller/follow_joint_trajectory",
      [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const Follow::Goal>) {
        if (!action_client_->action_server_is_ready()) return rclcpp_action::GoalResponse::REJECT;
        ++high_goals_count_;
        gate_.action(true);  // Block low before accepting a public high goal.
        reason_ = "trajectory action";
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<ServerGoal> goal) {
        const auto it = high_goals_.find(goal->get_goal_id());
        if (it != high_goals_.end()) {
          it->second->cancel_requested = true;
          cancelHigh(it->second);
        }
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<ServerGoal> goal) {
        auto proxy = std::make_shared<ProxyGoal>();
        proxy->upstream = goal;
        high_goals_[goal->get_goal_id()] = proxy;
        after_low_.push_back([this, proxy]() { sendHigh(proxy); });
        flushHigh();
      });
    normal_sub_ = node_->create_subscription<Trajectory>("/iiwa_arm_controller/joint_trajectory", 10,
      [this](const Trajectory& msg) {
        // The low action must have a terminal controller result before ANY high
        // output. DDS publication/cancel acknowledgements alone are insufficient.
        gate_.normal(trajectoryDeadline(msg));
        reason_ = "ordinary trajectory";
        after_low_.push_back([this, msg]() {
          gate_.normal(trajectoryDeadline(msg));
          output_->publish(msg);
        });
        flushHigh();
      });
    priority_ = node_->create_service<Priority>("/cobot/tcp_gizmo/priority",
      [this](const std::shared_ptr<rmw_request_id_t> header, const Priority::Request::SharedPtr request) {
        const auto token = ++lease_requests_[request->owner];
        if (!request->acquire) {
          gate_.release(request->owner);
          replyPriority(header, true, "priority released");
          return;
        }
        if (!gate_.acquire(request->owner, request->ttl, steady())) {
          replyPriority(header, false, "owner and finite TTL in (0, 60] required");
          return;
        }
        reason_ = "waiting for low action to stop";
        after_low_.push_back([this, header, request, token]() {
          if (lease_requests_[request->owner] != token) {
            replyPriority(header, false, "priority request superseded"); return;
          }
          // TTL starts at acknowledgement, even if controller completion was
          // delayed. The pending callback itself prevents any new low command.
          gate_.acquire(request->owner, request->ttl, steady());
          replyPriority(header, true, "controller low action stopped; priority granted");
          reason_ = "motion priority lease";
        });
        flushHigh();
      });
    timer_ = node_->create_wall_timer(std::chrono::duration<double>(period_), [this]() { tick(); });
  }

private:
  template<class T> T param(const std::string& name, const T& value) {
    if (!node_->has_parameter(name)) return node_->declare_parameter<T>(name, value);
    return node_->get_parameter(name).get_value<T>();
  }
  double positive(const std::string& name, double value) {
    value = param<double>(name, value);
    if (!std::isfinite(value) || value <= 0.) throw std::runtime_error("Invalid " + name);
    return value;
  }
  static double steady() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  moveit::core::RobotStatePtr current() {
    auto monitor = scene_->getStateMonitor();
    if (!monitor || !monitor->haveCompleteState(rclcpp::Duration::from_seconds(state_timeout_))) return {};
    auto pair = monitor->getCurrentStateAndTime();
    const double age = (node_->now() - pair.second).seconds();
    if (age < -.01 || age > state_timeout_) return {};
    const auto stamp = pair.second.nanoseconds();
    if (stamp != feedback_stamp_) { feedback_stamp_ = stamp; feedback_received_ = steady(); }
    if (steady() - feedback_received_ > state_timeout_) return {};
    pair.first->update();
    std::vector<double> positions;
    pair.first->copyJointGroupPositions(group_, positions);
    if (!std::all_of(positions.begin(), positions.end(), [](double x) { return std::isfinite(x); })) return {};
    return pair.first;
  }
  bool settled(const moveit::core::RobotState& state) {
    std::vector<double> positions;
    state.copyJointGroupPositions(group_, positions);
    const double now = node_->now().seconds();
    if (previous_.size() != positions.size() || now < previous_time_) {
      previous_ = positions; previous_time_ = now; stable_since_ = now; return false;
    }
    if (now > previous_time_) {
      double delta = 0.;
      for (std::size_t i = 0; i < positions.size(); ++i) delta = std::max(delta, std::abs(positions[i] - previous_[i]));
      if (delta / (now - previous_time_) > .01) stable_since_ = now;
      previous_ = positions; previous_time_ = now;
    }
    return now - stable_since_ >= .15;
  }
  double trajectoryDeadline(const Trajectory& msg) const {
    double start = node_->now().seconds();
    if (rclcpp::Time(msg.header.stamp).nanoseconds() != 0)
      start = std::max(start, rclcpp::Time(msg.header.stamp).seconds());
    double duration = 0.;
    for (const auto& point : msg.points)
      duration = std::max(duration, rclcpp::Duration(point.time_from_start).seconds());
    return start + duration;
  }
  void replyPriority(const std::shared_ptr<rmw_request_id_t>& header, bool success, const std::string& message) {
    Priority::Response response;
    response.success = success; response.message = message;
    priority_->send_response(*header, response);
  }
  void flushHigh() {
    if (!low_barrier_.idle()) return;
    while (!after_low_.empty()) {
      auto callback = std::move(after_low_.front());
      after_low_.pop_front();
      callback();
    }
  }
  void sendLow(const Trajectory& trajectory) {
    if (!action_client_->action_server_is_ready()) { reject("controller action unavailable"); return; }
    if (!low_barrier_.begin()) return;
    Follow::Goal goal;
    goal.trajectory = trajectory;
    // One released marker pose is one complete trajectory. The user chose
    // to finish this trajectory before dispatching a queued ordinary command.
    // No low CancelGoal request is needed (including the JTC cancel/result race).
    goal.goal_time_tolerance = rclcpp::Duration::from_seconds(state_timeout_);
    rclcpp_action::Client<Follow>::SendGoalOptions options;
    options.goal_response_callback = [this](const ClientGoal::SharedPtr handle) {
      low_goal_ = handle;
      low_barrier_.accepted(static_cast<bool>(handle));
      if (!handle) {
        gate_.invalidate(); reason_ = "controller rejected low trajectory"; flushHigh();
      }
    };
    options.result_callback = [this](const ClientGoal::WrappedResult& result) {
      low_goal_.reset(); low_barrier_.terminal();
      // A new epoch acknowledges completion to the marker and permits a new
      // gesture. Never redispatch the same goal after a successful result.
      gate_.invalidate();
      reason_ = result.code == rclcpp_action::ResultCode::SUCCEEDED ?
        "target reached" : "controller did not complete TCP trajectory";
      flushHigh();
    };
    try { action_client_->async_send_goal(goal, options); }
    catch (const std::exception& error) {
      // An exception leaves delivery uncertain. Fail closed: do not forward a
      // high command without a terminal controller result for this send.
      gate_.invalidate(); reason_ = std::string("low action send uncertain: ") + error.what();
    }
  }
  void cancelHigh(const std::shared_ptr<ProxyGoal>& proxy) {
    if (proxy->downstream && proxy->cancel_requested && !proxy->cancel_sent && !proxy->finished) {
      proxy->cancel_sent = true;
      action_client_->async_cancel_goal(proxy->downstream);
    }
  }
  void finishHigh(const std::shared_ptr<ProxyGoal>& proxy, rclcpp_action::ResultCode code,
                  const std::shared_ptr<Follow::Result>& result) {
    if (proxy->finished) return;
    proxy->finished = true;
    if (proxy->upstream->is_canceling()) proxy->upstream->canceled(result);
    else if (code == rclcpp_action::ResultCode::SUCCEEDED) proxy->upstream->succeed(result);
    else proxy->upstream->abort(result);
    high_goals_.erase(proxy->upstream->get_goal_id());
    if (high_goals_count_ > 0) --high_goals_count_;
    gate_.action(high_goals_count_ > 0);
  }
  void sendHigh(const std::shared_ptr<ProxyGoal>& proxy) {
    if (proxy->cancel_requested || proxy->upstream->is_canceling()) {
      auto result = std::make_shared<Follow::Result>();
      result->error_string = "canceled before controller dispatch";
      finishHigh(proxy, rclcpp_action::ResultCode::CANCELED, result); return;
    }
    if (!action_client_->action_server_is_ready()) {
      auto result = std::make_shared<Follow::Result>();
      result->error_code = Follow::Result::INVALID_GOAL;
      result->error_string = "controller action unavailable";
      finishHigh(proxy, rclcpp_action::ResultCode::ABORTED, result); return;
    }
    rclcpp_action::Client<Follow>::SendGoalOptions options;
    options.goal_response_callback = [this, proxy](const ClientGoal::SharedPtr handle) {
      proxy->downstream = handle;
      if (!handle) {
        auto result = std::make_shared<Follow::Result>();
        result->error_code = Follow::Result::INVALID_GOAL;
        result->error_string = "controller rejected trajectory";
        finishHigh(proxy, rclcpp_action::ResultCode::ABORTED, result);
      } else cancelHigh(proxy);
    };
    options.feedback_callback = [proxy](ClientGoal::SharedPtr, const std::shared_ptr<const Follow::Feedback> feedback) {
      if (!proxy->finished && proxy->upstream->is_active()) proxy->upstream->publish_feedback(std::make_shared<Follow::Feedback>(*feedback));
    };
    options.result_callback = [this, proxy](const ClientGoal::WrappedResult& result) {
      finishHigh(proxy, result.code, result.result);
    };
    try { action_client_->async_send_goal(*proxy->upstream->get_goal(), options); }
    catch (const std::exception& error) {
      // Keep its high reservation on ambiguous delivery; exposing low motion
      // here could overwrite a goal that the controller actually received.
      reason_ = std::string("high action send uncertain: ") + error.what();
    }
  }
  void reject(const std::string& reason) {
    RCLCPP_WARN(node_->get_logger(), "TCP gizmo rejected target: %s", reason.c_str());
    gate_.invalidate(); reason_ = reason;
  }
  bool safe(moveit::core::RobotState& state, const planning_scene::PlanningSceneConstPtr& scene) {
    state.update();
    return state.satisfiesBounds(group_) && !scene->isStateColliding(state, group_name_);
  }
  void command(const iiwa_msgs::msg::GizmoCommand& msg) {
    auto state = current();
    if (!state) { reject("joint state unavailable or stale"); return; }
    if (!after_low_.empty() || !gate_.allowed(steady(), node_->now().seconds(), settled(*state))) return;
    if (msg.epoch != gate_.epoch()) return;
    if (!low_barrier_.idle()) return;
    const auto& p = msg.target.pose;
    const double age = (node_->now() - rclcpp::Time(msg.target.header.stamp)).seconds();
    Eigen::Quaterniond q(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
    Eigen::Vector3d t(p.position.x, p.position.y, p.position.z);
    if (msg.target.header.frame_id != frame_ || !q.coeffs().allFinite() || !t.allFinite() ||
        std::abs(q.norm() - 1.) > .01 || age < -.01 || age > command_timeout_) {
      reject("invalid or stale target"); return;
    }
    Eigen::Isometry3d requested = Eigen::Isometry3d::Identity();
    requested.linear() = q.normalized().toRotationMatrix(); requested.translation() = t;
    const auto actual = state->getGlobalLinkTransform(frame_).inverse() * state->getGlobalLinkTransform(tcp_);
    if (!requiresGizmoMotion((requested.translation() - actual.translation()).norm(),
                            Eigen::Quaterniond(requested.linear()).angularDistance(Eigen::Quaterniond(actual.linear())))) {
      gate_.invalidate();
      reason_ = "target reached"; return;
    }
    const auto model_target = state->getGlobalLinkTransform(frame_) * requested;
    // Solve once on release. setFromIK handles the tool's fixed TCP offset.
    moveit::core::RobotState candidate(*state);
    planning_scene_monitor::LockedPlanningSceneRO scene(scene_);
    const bool valid = candidate.setFromIK(group_, model_target, tcp_, .01,
      [this, &scene](moveit::core::RobotState* s, const moveit::core::JointModelGroup* group, const double* values) {
        s->setJointGroupPositions(group, values); return safe(*s, scene);
      });
    if (!valid || !safe(candidate, scene)) { reject("target unreachable, outside limits or colliding"); return; }
    // Do not accept approximate IK solutions as reachable targets.
    const auto reached = candidate.getGlobalLinkTransform(tcp_);
    if ((reached.translation() - model_target.translation()).norm() > .002 ||
        Eigen::AngleAxisd(reached.linear().transpose() * model_target.linear()).angle() > .01) {
      reject("IK solution does not reach target"); return;
    }
    candidate.copyJointGroupPositions(group_, ik_target_);
    gate_.accept(msg.epoch, steady(), command_timeout_);
    reason_ = "tracking";
  }
  void tick() {
    flushHigh();
    auto state = current();
    if (!state) {
      if (gate_.hasTarget()) reject("joint state unavailable or stale");
      publishState({}, false, "joint state unavailable or stale"); return;
    }
    const bool available = after_low_.empty() && gate_.allowed(steady(), node_->now().seconds(), settled(*state));
    if (!available) {
      publishState(state, false, reason_); return;
    }
    // command_timeout validates the age at receipt. A released goal is a
    // committed action and needs no stream of keepalive mouse commands.
    if (gate_.hasTarget() && low_barrier_.idle()) {
      try { step(state); }
      catch (const std::exception& e) { reject(std::string("TCP trajectory error: ") + e.what()); }
    }
    publishState(state, true, reason_);
  }
  void step(const moveit::core::RobotStatePtr& state) {
    const auto& names = group_->getVariableNames();
    if (ik_target_.size() != names.size()) { reject("missing IK target"); return; }
    std::vector<double> actual;
    state->copyJointGroupPositions(group_, actual);
    Eigen::VectorXd delta(names.size());
    double duration = 2. * period_;
    double max_travel = 0.;
    for (std::size_t i = 0; i < names.size(); ++i) {
      delta[i] = ik_target_[i] - actual[i];
      const double travel = std::abs(delta[i]);
      max_travel = std::max(max_travel, travel);
      const auto& limit = state->getRobotModel()->getVariableBounds(names[i]);
      // q(u) = q0 + (3u^2 - 2u^3) * delta, with zero endpoint velocities.
      if (limit.velocity_bounded_)
        duration = std::max(duration, 1.5 * travel / limit.max_velocity_);
      if (limit.acceleration_bounded_)
        duration = std::max(duration, std::sqrt(6. * travel / limit.max_acceleration_));
    }
    if (!delta.allFinite() || !std::isfinite(duration)) { reject("invalid trajectory limits"); return; }

    // Validate the complete path, including the actual tool tip. Use forward
    // Jacobians only to measure Cartesian speed; no inverse/singularity gate.
    // Densely sample the cubic path and increase duration to obey BOTH TCP
    // translation and rotation limits throughout it, not just at its endpoints.
    const Eigen::Vector3d tcp_offset = (state->getGlobalLinkTransform(tip_).inverse() *
                                        state->getGlobalLinkTransform(tcp_)).translation();
    const auto* tip_link = state->getRobotModel()->getLinkModel(tip_);
    const int samples = std::max(100, static_cast<int>(std::ceil(max_travel / .01)));
    planning_scene_monitor::LockedPlanningSceneRO scene(scene_);
    moveit::core::RobotState sample(*state);
    Eigen::MatrixXd jacobian;
    std::vector<double> positions(actual.size());
    for (int k = 0; k <= samples; ++k) {
      const double u = static_cast<double>(k) / samples;
      const double blend = u * u * (3. - 2. * u);
      for (std::size_t i = 0; i < positions.size(); ++i) positions[i] = actual[i] + blend * delta[i];
      sample.setJointGroupPositions(group_, positions);
      if (!safe(sample, scene)) { reject("TCP trajectory is outside limits or colliding"); return; }
      if (!sample.getJacobian(group_, tip_link, tcp_offset, jacobian)) {
        reject("cannot compute tool TCP speed"); return;
      }
      const Eigen::VectorXd twist = jacobian * delta * (6. * u * (1. - u));
      duration = std::max(duration, 1.01 * twist.head<3>().norm() / linear_speed_);
      duration = std::max(duration, 1.01 * twist.tail<3>().norm() / angular_speed_);
    }
    if (!std::isfinite(duration)) { reject("invalid TCP trajectory duration"); return; }
    Trajectory message;
    message.joint_names = names;
    trajectory_msgs::msg::JointTrajectoryPoint start, end;
    start.positions = actual;
    start.velocities.assign(actual.size(), 0.);
    start.time_from_start = rclcpp::Duration::from_seconds(.001);
    end.positions = ik_target_;
    end.velocities.assign(actual.size(), 0.);
    end.time_from_start = rclcpp::Duration::from_seconds(duration + .001);
    message.points = {start, end};
    reason_ = "executing released TCP target";
    sendLow(message);
  }
  void publishState(const moveit::core::RobotStatePtr& state, bool enabled, const std::string& reason) {
    iiwa_msgs::msg::GizmoState message;
    message.actual.header.frame_id = (state || have_actual_) ? frame_ : "";
    message.actual.header.stamp = node_->now();
    message.actual.pose = last_actual_;
    if (state) {
      const auto pose = state->getGlobalLinkTransform(frame_).inverse() * state->getGlobalLinkTransform(tcp_);
      message.actual.pose.position.x = pose.translation().x(); message.actual.pose.position.y = pose.translation().y();
      message.actual.pose.position.z = pose.translation().z();
      const Eigen::Quaterniond q(pose.linear());
      message.actual.pose.orientation.x = q.x(); message.actual.pose.orientation.y = q.y();
      message.actual.pose.orientation.z = q.z(); message.actual.pose.orientation.w = q.w();
      last_actual_ = message.actual.pose; have_actual_ = true;
    }
    message.enabled = enabled; message.epoch = gate_.epoch(); message.reason = reason;
    state_pub_->publish(message);
  }
  rclcpp::Node::SharedPtr node_;
  planning_scene_monitor::PlanningSceneMonitorPtr scene_;
  std::shared_ptr<servo::ParamListener> listener_;
  const moveit::core::JointModelGroup* group_{};
  std::string group_name_, tcp_, tip_, frame_, reason_{"ready"};
  double period_{}, command_timeout_{}, state_timeout_{}, linear_speed_{}, angular_speed_{};
  GizmoPolicy gate_;
  std::vector<double> ik_target_;
  bool have_actual_{false};
  LowGoalBarrier low_barrier_;
  ClientGoal::SharedPtr low_goal_;
  std::deque<std::function<void()>> after_low_;
  std::map<std::string, std::uint64_t> lease_requests_;
  std::map<rclcpp_action::GoalUUID, std::shared_ptr<ProxyGoal>> high_goals_;
  std::size_t high_goals_count_{0};
  rclcpp_action::Client<Follow>::SharedPtr action_client_;
  rclcpp_action::Server<Follow>::SharedPtr action_server_;
  std::vector<double> previous_;
  double previous_time_{0.}, stable_since_{0.}, feedback_received_{0.};
  std::int64_t feedback_stamp_{-1};
  geometry_msgs::msg::Pose last_actual_{};
  rclcpp::Publisher<Trajectory>::SharedPtr output_;
  rclcpp::Publisher<iiwa_msgs::msg::GizmoState>::SharedPtr state_pub_;
  rclcpp::Subscription<iiwa_msgs::msg::GizmoCommand>::SharedPtr command_sub_;
  rclcpp::Subscription<Trajectory>::SharedPtr normal_sub_;
  rclcpp::Service<Priority>::SharedPtr priority_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace iiwa_planning

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("tcp_gizmo", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  try {
    iiwa_planning::TcpGizmo gizmo(node);
    // All motion, priority, target and timer callbacks are serialized.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node); executor.spin();
  } catch (const std::exception& error) {
    RCLCPP_FATAL(node->get_logger(), "%s", error.what()); rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
