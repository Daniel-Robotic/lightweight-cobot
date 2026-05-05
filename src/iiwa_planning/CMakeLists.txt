cmake_minimum_required(VERSION 3.8)
project(iiwa_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(iiwa_msgs REQUIRED)

add_executable(motion_planning_cpp src/motion_planning_cpp.cpp)
target_include_directories(motion_planning_cpp PUBLIC include)

ament_target_dependencies(motion_planning_cpp
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  moveit_ros_planning
  moveit_msgs
  geometry_msgs
)


ament_python_install_package(${PROJECT_NAME})

install(TARGETS
  motion_planning_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/motion_planning_test.py
  # scripts/motion_planning.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME motion_planning
)

install(PROGRAMS
  scripts/move_to_pose_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME move_to_pose_server
)


ament_package()
