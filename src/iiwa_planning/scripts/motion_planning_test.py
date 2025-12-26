#!/usr/bin/env python3
import time

# generic ros libraries
import rclpy
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.logging import get_logger


def plan_and_execute(robot: MoveItPy,
                     planning_component,
                     logger: RcutilsLogger,
                     single_plan_parameters=None,
                     multi_plan_parameters=None,
                     sleep_time=0.0):
    logger.info("Planning trajectory")

    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(multi_plan_parameters=multi_plan_parameters)
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)


def main():
    rclpy.init()
    logger = get_logger("moveit_py.pose_goal")

    iiwa = MoveItPy(node_name="moveit_py")
    iiwa_arm = iiwa.get_planning_component("iiwa_arm")
    logger.info("MoveItPy instance created")

    iiwa_arm.set_start_state(configuration_name="ready")
    iiwa_arm.set_goal_state(configuration_name="extended")

    plan_and_execute(iiwa, iiwa_arm, logger, sleep_time=3.0)

    robot_model = iiwa.get_robot_model()
    robot_state = RobotState(robot_model)

    # randomize the robot state
    robot_state.set_to_random_positions()

    # set plan start state to current state
    iiwa_arm.set_start_state_to_current_state()

    # set goal state to the initialized robot state
    logger.info("Set goal state to the initialized robot state")
    iiwa_arm.set_goal_state(robot_state=robot_state)

    # plan to goal
    plan_and_execute(iiwa, iiwa_arm, logger, sleep_time=3.0)

    # set plan start state to current state
    iiwa_arm.set_start_state_to_current_state()

    # set pose goal with PoseStamped message
    from geometry_msgs.msg import PoseStamped

    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "base_link"
    pose_goal.pose.orientation.w = 1.0
    pose_goal.pose.position.x = 0.28
    pose_goal.pose.position.y = -0.2
    pose_goal.pose.position.z = 0.5
    iiwa_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="link7")

    # plan to goal
    plan_and_execute(iiwa, iiwa_arm, logger, sleep_time=3.0)


if __name__ == "__main__":
    main()