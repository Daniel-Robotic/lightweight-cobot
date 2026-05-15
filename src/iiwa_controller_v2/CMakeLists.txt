cmake_minimum_required(VERSION 3.8)
project(iiwa_controller_v2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)

# ── FRI SDK (shared with iiwa_controller via symlink) ─────────────────────────
set(FRI_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/libFRI)

file(GLOB_RECURSE FRI_SOURCES
  "${FRI_SDK_DIR}/src/base/*.cpp"
  "${FRI_SDK_DIR}/src/client_lbr/*.cpp"
  "${FRI_SDK_DIR}/src/client_trafo/*.cpp"
  "${FRI_SDK_DIR}/src/connection/*.cpp"
  "${FRI_SDK_DIR}/src/nanopb-0.2.8/*.c"
  "${FRI_SDK_DIR}/src/protobuf/*.c"
  "${FRI_SDK_DIR}/src/protobuf/*.cpp"
  "${FRI_SDK_DIR}/src/protobuf_gen/*.c"
)

add_library(fri_client_sdk_v2 STATIC ${FRI_SOURCES})

target_include_directories(fri_client_sdk_v2 PUBLIC
  ${FRI_SDK_DIR}/include
  ${FRI_SDK_DIR}/src/nanopb-0.2.8
  ${FRI_SDK_DIR}/src/protobuf
  ${FRI_SDK_DIR}/src/protobuf_gen
  ${FRI_SDK_DIR}/src/base
  ${FRI_SDK_DIR}/src/client_lbr
  ${FRI_SDK_DIR}/src/connection
)

target_compile_definitions(fri_client_sdk_v2 PUBLIC PB_FIELD_16BIT)
target_compile_options(fri_client_sdk_v2 PRIVATE -fpermissive -w)
set_target_properties(fri_client_sdk_v2 PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(fri_client_sdk_v2 PUBLIC pthread)

# ── Plugin library ─────────────────────────────────────────────────────────────
add_library(${PROJECT_NAME} SHARED
  src/fri_client.cpp
  src/system_interface.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PRIVATE
  fri_client_sdk_v2
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
)

pluginlib_export_plugin_description_file(
  hardware_interface
  iiwa_hardware_interface_plugin.xml
)

# ── Install ────────────────────────────────────────────────────────────────────
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  controller_interface hardware_interface pluginlib rclcpp rclcpp_lifecycle realtime_tools)

ament_package()
