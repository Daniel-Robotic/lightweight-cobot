cmake_minimum_required(VERSION 3.8)
project(iiwa_controller)

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
find_package(std_msgs REQUIRED)

# FRI SDK
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

add_library(fri_client_sdk STATIC ${FRI_SOURCES})

target_include_directories(fri_client_sdk SYSTEM PUBLIC
  ${FRI_SDK_DIR}/include
  ${FRI_SDK_DIR}/src/nanopb-0.2.8
  ${FRI_SDK_DIR}/src/protobuf
  ${FRI_SDK_DIR}/src/protobuf_gen
  ${FRI_SDK_DIR}/src/base
  ${FRI_SDK_DIR}/src/client_lbr
  ${FRI_SDK_DIR}/src/connection
)

target_compile_definitions(fri_client_sdk PUBLIC PB_FIELD_16BIT)
target_compile_options(fri_client_sdk PRIVATE -fpermissive -w)
set_target_properties(fri_client_sdk PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(fri_client_sdk PUBLIC pthread)

# Плагины hardware interface + controller
add_library(${PROJECT_NAME} SHARED
  src/FRIClient.cpp
  src/IIWAHardwareInterface.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PRIVATE
  fri_client_sdk
  controller_interface::controller_interface
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  realtime_tools::realtime_tools
  ${std_msgs_TARGETS}
)

pluginlib_export_plugin_description_file(
  hardware_interface
  iiwa_hardware_interface_plugin.xml
)

# Установка — только библиотека и заголовки, без config/launch/urdf
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
  controller_interface hardware_interface pluginlib rclcpp rclcpp_lifecycle realtime_tools std_msgs)

if(BUILD_TESTING)
  # Manual bounded loopback integration harness; no physical hardware access.
  add_executable(fake_fri_server test/fake_fri_server.cpp)
  target_link_libraries(fake_fri_server fri_client_sdk)
  add_library(fri_loopback_bind SHARED test/loopback_bind.cpp)
  target_link_libraries(fri_loopback_bind dl)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_fri_client test/test_fri_client.cpp)
  target_link_libraries(test_fri_client ${PROJECT_NAME} fri_client_sdk)
  ament_add_gtest(test_hardware test/test_hardware.cpp)
  target_link_libraries(test_hardware ${PROJECT_NAME} fri_client_sdk hardware_interface::hardware_interface)
endif()

ament_package()
