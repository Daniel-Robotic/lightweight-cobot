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

include_directories(include)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  # scripts/motion_planning_test.py
  scripts/motion_planning.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME motion_planning
)


ament_package()
