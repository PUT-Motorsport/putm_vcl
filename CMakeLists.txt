cmake_minimum_required(VERSION 3.8)

project(putm_pm09_vcl)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

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

add_executable(amk_rx_bridge src/amk_bridge/amk_rx_bridge.cpp)
target_include_directories(
  amk_rx_bridge
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(amk_rx_bridge rclcpp)
target_link_libraries(amk_rx_bridge "${cpp_typesupport_target}")

add_executable(amk_tx_bridge src/amk_bridge/amk_tx_bridge.cpp)
target_include_directories(
  amk_tx_bridge
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(amk_tx_bridge rclcpp)
target_link_libraries(amk_tx_bridge "${cpp_typesupport_target}")

add_executable(can_rx_bridge src/can_bridge/can_rx_bridge.cpp)
target_include_directories(
  can_rx_bridge
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(can_rx_bridge rclcpp)
target_link_libraries(can_rx_bridge "${cpp_typesupport_target}")

add_executable(can_tx_bridge src/can_bridge/can_tx_bridge.cpp)
target_include_directories(
  can_tx_bridge
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(can_tx_bridge rclcpp)
target_link_libraries(can_tx_bridge "${cpp_typesupport_target}")

add_executable(rtd_node src/rtd_node/rtd_node.cpp)
target_include_directories(
  rtd_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(rtd_node rclcpp)
target_link_libraries(rtd_node "${cpp_typesupport_target}")

install(TARGETS amk_node 
                amk_rx_bridge
                amk_tx_bridge
                can_tx_bridge
                can_rx_bridge
                rtd_node
        DESTINATION lib/${PROJECT_NAME}/)

install(DIRECTORY launch/ 
        DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
