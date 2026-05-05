#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from moveit.planning import MoveItPy, PlanningComponent
from iiwa_msgs.srv import MoveToPose


POSE_LINK = "link_ee"
PLANNING_GROUP = "iiwa_arm"


def main(args=None):
    rclpy.init(args=args)

    # MoveItPy создаёт C++ узел с именем из лонча → читает robot_description_kinematics и т.д.
    moveit = MoveItPy(node_name="move_to_pose_server")
    arm: PlanningComponent = moveit.get_planning_component(PLANNING_GROUP)

    # Отдельный лёгкий узел для сервиса — другое имя, нет конфликта параметров
    node = Node("move_to_pose_service")
    logger = node.get_logger()

    cb_group = ReentrantCallbackGroup()

    def handle(request: MoveToPose.Request, response: MoveToPose.Response):
        pose = request.pose
        if not pose.header.frame_id:
            pose.header.frame_id = "base_link"

        arm.set_start_state_to_current_state()
        arm.set_goal_state(pose_stamped_msg=pose, pose_link=POSE_LINK)

        logger.info(
            f"Planning to ({pose.pose.position.x:.3f}, "
            f"{pose.pose.position.y:.3f}, {pose.pose.position.z:.3f})"
        )
        plan_result = arm.plan()

        if not plan_result:
            response.success = False
            response.message = "Planning failed: pose may be unreachable or in collision"
            logger.error(response.message)
            return response

        moveit.execute(plan_result.trajectory, controllers=[])
        response.success = True
        response.message = "Motion executed successfully"
        logger.info(response.message)
        return response

    node.create_service(MoveToPose, "iiwa/move_to_pose", handle, callback_group=cb_group)
    logger.info(f"MoveToPoseServer ready (pose_link={POSE_LINK})")

    try:
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
