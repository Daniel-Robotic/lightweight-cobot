cmake_minimum_required(VERSION 3.8)
project(iiwa_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(iiwa_msgs REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

add_executable(tcp_gizmo src/tcp_gizmo.cpp)
target_compile_features(tcp_gizmo PUBLIC cxx_std_17)
target_include_directories(tcp_gizmo PRIVATE include)
ament_target_dependencies(tcp_gizmo rclcpp rclcpp_action control_msgs iiwa_msgs geometry_msgs trajectory_msgs
  action_msgs moveit_servo moveit_ros_planning moveit_core tf2_eigen)

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
  tcp_gizmo
  motion_planning_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/move_to_pose_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME move_to_pose_server
)

install(PROGRAMS
  scripts/motion_sequence_runner.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME motion_sequence_runner
)

install(FILES
  scripts/motion_sequence_config.json
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_tcp_gizmo_policy test/test_tcp_gizmo_policy.cpp)
  target_compile_features(test_tcp_gizmo_policy PUBLIC cxx_std_17)
  target_include_directories(test_tcp_gizmo_policy PRIVATE include)
  ament_add_pytest_test(test_motion_priority test/test_motion_priority.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR})
endif()

ament_package()
