#!/usr/bin/env python3

import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException

from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
from std_srvs.srv import Trigger

from moveit.planning import MoveItPy, PlanningComponent, PlanRequestParameters
from moveit.core.robot_state import RobotState

from iiwa_msgs.action import MoveToPose, MoveToJoints
from iiwa_msgs.srv import MoveToNamedPose


# Таблица планировщиков
PLANNERS = {
    "ompl":  ("ompl", "RRTConnectkConfigDefault", 10.0),
    "ptp":   ("pilz_industrial_motion_planner", "PTP", 2.0),
    "lin":   ("pilz_industrial_motion_planner", "LIN", 2.0),
    "circ":  ("pilz_industrial_motion_planner", "CIRC", 2.0),
    "chomp": ("chomp", "", 10.0),
}


def _abc_to_quaternion(a: float, b: float, c: float):
    """ZYX Euler (радианы, конвенция KUKA ABC) → (qx, qy, qz, qw)."""
    return Rotation.from_euler("ZYX", [a, b, c], degrees=False).as_quat()


class IiwaMotionServer(Node):
    """Сервер управления движением манипулятора iiwa.

    Предоставляет:
      - action iiwa/move_to_pose    — перемещение в декартову позу
      - action iiwa/move_to_joints  — перемещение по суставным координатам
      - service iiwa/move_to_named  — перемещение в именованную позу из SRDF
      - service iiwa/stop           — немедленная остановка движения
    """

    def __init__(self):
        super().__init__("iiwa_motion_server")
        self._setup_parameters()
        self._setup_moveit()
        self._setup_servers()

        self.get_logger().info("Сервер движения iiwa запущен")
        self.get_logger().info(f"  pose_link={self._pose_link}  group={self._planning_group}")

    def _setup_parameters(self):
        self.declare_parameter("pose_link", "link_ee")
        self.declare_parameter("planning_group", "iiwa_arm")
        self.declare_parameter("default_frame", "base_link")
        self.declare_parameter("default_planner", "ompl")
        self.declare_parameter("planning_attempts", 3)

        self._pose_link = self.get_parameter("pose_link").value
        self._planning_group = self.get_parameter("planning_group").value
        self._default_frame = self.get_parameter("default_frame").value
        self._default_planner = self.get_parameter("default_planner").value
        self._planning_attempts = self.get_parameter("planning_attempts").value

    def _setup_moveit(self):
        self._moveit = MoveItPy(node_name="iiwa_motion_server")
        self._arm: PlanningComponent = self._moveit.get_planning_component(self._planning_group)
        self._robot_model = self._moveit.get_robot_model()

    def _setup_servers(self):
        cb = ReentrantCallbackGroup()

        ActionServer(
            self, MoveToPose, "iiwa/move_to_pose", self._execute_pose,
            callback_group=cb,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )
        ActionServer(
            self, MoveToJoints, "iiwa/move_to_joints", self._execute_joints,
            callback_group=cb,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )
        self.create_service(MoveToNamedPose, "iiwa/move_to_named", self._handle_named, callback_group=cb)
        self.create_service(Trigger, "iiwa/stop", self._handle_stop, callback_group=cb)

    def _make_plan_params(self, pipeline: str, planner_id: str, plan_time: float, velocity_scale: float, accel_scale: float | None = None) -> PlanRequestParameters:
        params = PlanRequestParameters(self._moveit, self._planning_group)
        params.planning_pipeline = pipeline
        params.planner_id = planner_id
        params.planning_time = plan_time
        params.planning_attempts = self._planning_attempts
        params.max_velocity_scaling_factor = velocity_scale
        params.max_acceleration_scaling_factor = accel_scale if accel_scale is not None else velocity_scale
        return params

    def _plan_and_execute(self, plan_params: PlanRequestParameters, goal_handle):
        """Планирует траекторию и выполняет её с поддержкой отмены.

        Возвращает (True, msg) при успехе, (False, msg) при ошибке,
        (None, 'canceled') если цель была отменена.
        """
        plan_result = self._arm.plan(single_plan_parameters=plan_params)
        if not plan_result:
            return False, "Планирование не удалось: поза недостижима или в столкновении"

        if goal_handle.is_cancel_requested:
            return None, "canceled"

        done = threading.Event()
        failed = threading.Event()

        def do_execute():
            try:
                self._moveit.execute(plan_result.trajectory, controllers=[])
            except Exception as exc:
                self.get_logger().error(f"Ошибка выполнения траектории: {exc}")
                failed.set()
            finally:
                done.set()

        threading.Thread(target=do_execute, daemon=True).start()

        while not done.wait(timeout=0.05):
            if goal_handle.is_cancel_requested:
                try:
                    self._moveit.get_trajectory_execution_manager().stop_execution()
                except Exception:
                    pass
                done.wait()
                return None, "canceled"

        if failed.is_set():
            return False, "Выполнение траектории завершилось ошибкой"

        return True, "Движение выполнено успешно"

    def _finish_action(self, goal_handle, result, ok, msg):
        """Устанавливает финальное состояние action goal и заполняет result."""
        result.success = bool(ok)
        result.message = msg
        if ok is None:
            goal_handle.canceled()
        elif ok:
            goal_handle.succeed()
        else:
            self.get_logger().error(msg)
            goal_handle.abort()
        return result

    def _execute_pose(self, goal_handle):
        req = goal_handle.request
        feedback = MoveToPose.Feedback()
        result = MoveToPose.Result()

        velocity_scale = max(0.01, min(1.0, float(req.speed)))
        planner_key = (req.planner or self._default_planner).lower()

        if planner_key not in PLANNERS:
            result.success = False
            result.message = f"Неизвестный планировщик '{planner_key}'. Доступные: {', '.join(PLANNERS)}"
            self.get_logger().error(result.message)
            goal_handle.abort()
            return result

        pipeline, planner_id, plan_time = PLANNERS[planner_key]

        pose = PoseStamped()
        pose.header.frame_id = req.frame_id or self._default_frame
        pose.pose.position.x = req.x
        pose.pose.position.y = req.y
        pose.pose.position.z = req.z
        qx, qy, qz, qw = _abc_to_quaternion(req.a, req.b, req.c)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.get_logger().info(
            f"[pose] xyz=({req.x:.3f}, {req.y:.3f}, {req.z:.3f})  "
            f"abc=({req.a:.3f}, {req.b:.3f}, {req.c:.3f}) рад  "
            f"speed={velocity_scale:.2f}  planner={planner_key}"
        )

        feedback.state = "planning"
        goal_handle.publish_feedback(feedback)

        self._arm.set_start_state_to_current_state()
        self._arm.set_goal_state(pose_stamped_msg=pose, pose_link=self._pose_link)

        feedback.state = "executing"
        goal_handle.publish_feedback(feedback)

        plan_params = self._make_plan_params(pipeline, planner_id, plan_time, velocity_scale)
        ok, msg = self._plan_and_execute(plan_params, goal_handle)
        return self._finish_action(goal_handle, result, ok, msg)

    def _execute_joints(self, goal_handle):
        req = goal_handle.request
        feedback = MoveToJoints.Feedback()
        result = MoveToJoints.Result()

        velocity_scale = max(0.01, min(1.0, float(req.speed)))
        joints = list(req.joints)

        self.get_logger().info(
            f"[joints] {[f'{v:.3f}' for v in joints]}  speed={velocity_scale:.2f}"
        )

        feedback.state = "planning"
        goal_handle.publish_feedback(feedback)

        # Формируем целевое состояние по суставным координатам
        goal_state = RobotState(self._robot_model)
        goal_state.set_joint_group_positions(self._planning_group, joints)
        goal_state.update()

        self._arm.set_start_state_to_current_state()
        self._arm.set_goal_state(robot_state=goal_state)

        feedback.state = "executing"
        goal_handle.publish_feedback(feedback)

        plan_params = self._make_plan_params(
            "ompl", "RRTConnectkConfigDefault", 10.0, velocity_scale
        )
        ok, msg = self._plan_and_execute(plan_params, goal_handle)
        return self._finish_action(goal_handle, result, ok, msg)

    def _handle_named(self, request: MoveToNamedPose.Request, response: MoveToNamedPose.Response):
        name = request.name.strip()
        velocity_scale = max(0.01, min(1.0, float(request.speed)))

        self.get_logger().info(f"[named] name='{name}'  speed={velocity_scale:.2f}")

        self._arm.set_start_state_to_current_state()
        try:
            self._arm.set_goal_state(configuration_name=name)
        except Exception as exc:
            response.success = False
            response.message = f"Неизвестное состояние '{name}': {exc}"
            self.get_logger().error(response.message)
            return response

        plan_params = self._make_plan_params(
            "pilz_industrial_motion_planner", "PTP", 2.0, velocity_scale
        )
        plan_result = self._arm.plan(single_plan_parameters=plan_params)
        if not plan_result:
            response.success = False
            response.message = f"Не удалось построить траекторию для '{name}'"
            self.get_logger().error(response.message)
            return response

        self._moveit.execute(plan_result.trajectory, controllers=[])
        response.success = True
        response.message = f"Переместился в '{name}'"
        self.get_logger().info(response.message)
        return response

    def _handle_stop(self, request: Trigger.Request, response: Trigger.Response):
        try:
            self._moveit.get_trajectory_execution_manager().stop_execution()
            response.success = True
            response.message = "Выполнение траектории остановлено"
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        self.get_logger().info(f"[stop] {response.message}")
        return response


def main(args=None):
    try:
        rclpy.init(args=args)
        node = IiwaMotionServer()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
