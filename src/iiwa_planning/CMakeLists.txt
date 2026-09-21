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
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module)
find_package(pybind11 REQUIRED)
pybind11_add_module(_iiwa_spline_limits src/spline_limits_py.cpp)
target_compile_features(_iiwa_spline_limits PRIVATE cxx_std_17)
target_include_directories(_iiwa_spline_limits PRIVATE include)
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(TARGETS _iiwa_spline_limits LIBRARY DESTINATION ${PYTHON_INSTALL_DIR})

add_library(iiwa_spline_limits SHARED src/limit_spline_dynamics.cpp)
target_compile_features(iiwa_spline_limits PUBLIC cxx_std_17)
target_include_directories(iiwa_spline_limits PRIVATE include)
ament_target_dependencies(iiwa_spline_limits rclcpp moveit_core moveit_ros_planning pluginlib sensor_msgs)
pluginlib_export_plugin_description_file(moveit_core planning_response_plugins.xml)
install(TARGETS iiwa_spline_limits LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_spline_limits test/test_spline_limits.cpp)
  target_compile_features(test_spline_limits PRIVATE cxx_std_17)
  target_compile_options(test_spline_limits PRIVATE -UNDEBUG)
  target_include_directories(test_spline_limits PRIVATE include)
  add_test(NAME spline_limits COMMAND test_spline_limits)
  find_package(joint_trajectory_controller REQUIRED)
  add_executable(test_jtc_interpolation test/test_jtc_interpolation.cpp)
  target_compile_features(test_jtc_interpolation PRIVATE cxx_std_17)
  target_compile_options(test_jtc_interpolation PRIVATE -UNDEBUG)
  target_include_directories(test_jtc_interpolation PRIVATE include)
  ament_target_dependencies(test_jtc_interpolation joint_trajectory_controller)
  add_test(NAME jtc_interpolation COMMAND test_jtc_interpolation)
  add_executable(test_response_adapter test/test_response_adapter.cpp)
  target_compile_options(test_response_adapter PRIVATE -UNDEBUG)
  ament_target_dependencies(test_response_adapter moveit_core moveit_ros_planning pluginlib)
  add_test(NAME response_adapter COMMAND test_response_adapter)
  add_executable(test_start_state_monitor test/test_start_state_monitor.cpp)
  target_compile_options(test_start_state_monitor PRIVATE -UNDEBUG)
  target_include_directories(test_start_state_monitor PRIVATE include)
  ament_target_dependencies(test_start_state_monitor rclcpp sensor_msgs)
  add_test(NAME start_state_monitor COMMAND test_start_state_monitor)
  set_tests_properties(start_state_monitor PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=231;ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" TIMEOUT 15)
  add_test(NAME direct_trajectory COMMAND ${Python3_EXECUTABLE} -m pytest -q ${CMAKE_CURRENT_SOURCE_DIR}/test/test_prepare_trajectory.py)
  set_tests_properties(direct_trajectory PROPERTIES ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}:${CMAKE_CURRENT_SOURCE_DIR}:$ENV{PYTHONPATH}")
endif()

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


ament_package()
