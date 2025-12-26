#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, PlanningComponent
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from tf_transformations import quaternion_from_euler


class MotionPlaning(Node):
    def __init__(self):
        super().__init__("motion_planning_node")

        self.moveit = MoveItPy(node_name="motion_planning_node")
        self.robot_arm = self.moveit.get_planning_component("iiwa_arm")
        self.planning_scene_monitor = self.moveit.get_planning_scene_monitor()

        self.get_logger().warn("***********************************************************")
        self.get_logger().info("MotionPlanning node is ready...")
        self.get_logger().warn("***********************************************************")
        # self.robot_arm.set_start_state(configuration_name="home")
        # self.robot_arm.set_goal_state(configuration_name="work")
        # self.plan_end_execute()

        self.timer = self.create_timer(5.0, self._run_once)

    def _run_once(self):
        self.timer.cancel()
        self.handle_plan_execute_pose(0.677, 0.25, 0.21, 3.14, 0, 3.14)


    def handle_plan_execute_pose(self, x, y, z, a, b, c):
        
        pose_goal = PoseStamped()
        pose_goal.header.frame_id="base_link"
        pose_goal.header.stamp = self.get_clock().now().to_msg()
        pose_goal.pose.position.x=x
        pose_goal.pose.position.y=y
        pose_goal.pose.position.z=z

        xx, yy, zz, w = quaternion_from_euler(a, b, c)

        pose_goal.pose.orientation.x=xx
        pose_goal.pose.orientation.y=yy
        pose_goal.pose.orientation.z=zz
        pose_goal.pose.orientation.w=w

        # Check collisions
        with self.planning_scene_monitor.read_only() as scene:
            
            robot_state = scene.current_state
            original_joint_positions = robot_state.get_joint_group_positions("iiwa_arm")

            ok_ik = robot_state.set_from_ik("iiwa_arm", pose_goal.pose, "link_ee")
            robot_state.update() 

            if not ok_ik:
                self.get_logger().warn("***********************************************************")
                self.get_logger().error("IK failed -> abort")
                self.get_logger().warn("***********************************************************")
                return

            robot_collision_status = scene.is_state_colliding(
                robot_state=robot_state, 
                joint_model_group_name="iiwa_arm", 
                verbose=True
            )
            if robot_collision_status:
                self.get_logger().warn("***********************************************************")
                self.get_logger().error("Goal in collision -> abort")
                self.get_logger().warn("***********************************************************")
                return

            self.get_logger().warn("***********************************************************")
            self.get_logger().info(f"\nRobot is in collision: {robot_collision_status}\n")
            self.get_logger().warn("***********************************************************")

            robot_state.set_joint_group_positions(
                "iiwa_arm",
                original_joint_positions,
            )
            robot_state.update() 

        # получить текущее состояние как изначальное для перемещения
        self.robot_arm.set_start_state_to_current_state()
        # self.robot_arm.set_start_state(configuration_name="home")
        # self.robot_arm.set_goal_state(configuration_name="work")

        self.robot_arm.set_goal_state(
            pose_stamped_msg=pose_goal, 
            pose_link="link_ee"
        )
        self.plan_end_execute()


    def plan_end_execute(self):
        self.get_logger().warn("***********************************************************")
        self.get_logger().info("Planning trajectory")
        self.get_logger().warn("***********************************************************")

        plan_result = self.robot_arm.plan()
        if plan_result:
            self.get_logger().warn("***********************************************************")
            self.get_logger().info("Executing plan")
            self.get_logger().warn("***********************************************************")
            robot_trajectory = plan_result.trajectory
            self.moveit.execute(robot_trajectory, controllers=[])
        else:
            self.get_logger().warn("***********************************************************")
            self.get_logger().info("Planning failed")
            self.get_logger().warn("***********************************************************")
        

def main(args=None):
    try:
        rclpy.init(args=args)
        node = MotionPlaning()
        rclpy.spin(node=node)
        node.destroy_node()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()