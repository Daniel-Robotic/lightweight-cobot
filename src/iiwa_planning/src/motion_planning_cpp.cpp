#include <chrono>
#include <memory>
#include <string>

#include <pluginlib/class_loader.hpp>

#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/planning_pipeline/planning_pipeline.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/kinematic_constraints/utils.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>


class MotionPlanning: public rclcpp::Node 
{
    public:
        MotionPlanning() : Node("motion_planning_node",
                                rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        {
            RCLCPP_INFO(get_logger(), "***************");
            RCLCPP_INFO(get_logger(), "MotionPlaning node created");
            RCLCPP_INFO(get_logger(), "***************");

            timer_ = create_wall_timer(
                std::chrono::milliseconds(200),
                std::bind(&MotionPlanning::runOnce, this)
            );            

        }

    private:
        void runOnce() 
        {
            timer_->cancel();
            
        }

    private:
        rclcpp::TimerBase::SharedPtr timer_;

        const std::string planning_group_ = "iiwa_arm";
        const std::string base_frame_ = "base_link";
        const std::string ee_link = "link_ee";


};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionPlanning>());
    rclcpp::shutdown();
    return 0;
}
