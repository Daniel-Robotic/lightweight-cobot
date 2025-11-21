cmake_minimum_required(VERSION 3.8)
project(iiwa_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Eigen3 REQUIRED)

# FRI headers / sources
set(FRI_HEADER
    external/libFRI/include
    external/libFRI/src/protobuf_gen
    external/libFRI/src/nanopb-0.2.8
    external/libFRI/src/protobuf
    external/libFRI/src/connection
    external/libFRI/src/client_lbr
    external/libFRI/src/base
  )

set(FRI_SRC
    external/libFRI/src/base/friClientApplication.cpp
    external/libFRI/src/client_lbr/friLBRClient.cpp
    external/libFRI/src/client_lbr/friLBRCommand.cpp
    external/libFRI/src/client_lbr/friLBRState.cpp
    external/libFRI/src/connection/friUdpConnection.cpp
    external/libFRI/src/protobuf/friCommandMessageEncoder.cpp
    external/libFRI/src/protobuf/friMonitoringMessageDecoder.cpp
    external/libFRI/src/protobuf/pb_frimessages_callbacks.c
    external/libFRI/src/protobuf_gen/FRIMessages.pb.c
    external/libFRI/src/nanopb-0.2.8/pb_decode.c
    external/libFRI/src/nanopb-0.2.8/pb_encode.c
    external/libFRI/src/client_trafo/friTransformationClient.cpp
  )

add_library(${PROJECT_NAME}
  SHARED
  src/IIWAHardwareInterface.cpp
  ${FRI_SRC}
  src/FRIClient.cpp
)

target_include_directories(${PROJECT_NAME}
  PRIVATE
    include
    ${FRI_HEADER}
)

target_compile_definitions(${PROJECT_NAME}
  PRIVATE
    PB_FIELD_16BIT
    HAVE_SOCKLEN_T
    PB_FIELD_16BIT 
    PB_NO_ERRMSG
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
    hardware_interface::hardware_interface
    pluginlib::pluginlib
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    Eigen3::Eigen
)

# Export plugin description for pluginlib
pluginlib_export_plugin_description_file(hardware_interface iiwa_controller_plugin.xml)

# Installation
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  Eigen3
)

ament_package()
