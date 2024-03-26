cmake_minimum_required(VERSION 3.8)

project(putm_pm09_vcl)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME} 
  "msg/AmkStatus.msg" 
  "msg/AmkControl.msg" 
  "msg/AmkData.msg"
  "msg/Frontbox.msg"
  "msg/Dash.msg"
  "msg/Rtd.msg"
  "msg/Setpoints.msg")

ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}
                              rosidl_typesupport_cpp)

add_executable(amk_node src/amk_node/amk_node.cpp)
target_include_directories(
  amk_node PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(amk_node rclcpp)
target_link_libraries(amk_node "${cpp_typesupport_target}")

add_executable(amk_bridge_rx_node src/amk_bridge/amk_rx_bridge.cpp)
target_include_directories(
  amk_bridge_rx_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(amk_bridge_rx_node rclcpp)
target_link_libraries(amk_bridge_rx_node "${cpp_typesupport_target}")

add_executable(amk_bridge_tx_node src/amk_bridge/amk_tx_bridge.cpp)
target_include_directories(
  amk_bridge_tx_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(amk_bridge_tx_node rclcpp)
target_link_libraries(amk_bridge_tx_node "${cpp_typesupport_target}")

add_executable(can_rx_bridge_node src/can_bridge/can_rx_bridge.cpp)
target_include_directories(
  can_rx_bridge_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(can_rx_bridge_node rclcpp)
target_link_libraries(can_rx_bridge_node "${cpp_typesupport_target}")

add_executable(can_tx_bridge_node src/can_bridge/can_tx_bridge.cpp)
target_include_directories(
  can_tx_bridge_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(can_tx_bridge_node rclcpp)
target_link_libraries(can_tx_bridge_node "${cpp_typesupport_target}")

add_executable(joy_control_node src/can_node/joy_control.cpp)
target_include_directories(
  joy_control_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(joy_control_node rclcpp sensor_msgs)
target_link_libraries(joy_control_node "${cpp_typesupport_target}")

add_executable(rtd_node src/can_node/rtd.cpp)
target_include_directories(
  rtd_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(rtd_node rclcpp)
target_link_libraries(rtd_node "${cpp_typesupport_target}")

install(TARGETS amk_node amk_bridge_rx_node amk_bridge_tx_node
                can_tx_bridge_node can_rx_bridge_node joy_control_node
                rtd_node
        DESTINATION lib/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
