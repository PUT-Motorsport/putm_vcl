cmake_minimum_required(VERSION 3.8)

project(putm_vcl)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(putm_vcl_interfaces REQUIRED)

add_executable(amk_node src/amk_node/amk_node.cpp)
target_include_directories(
  amk_node PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                  $<INSTALL_INTERFACE:include>)
	  ament_target_dependencies(amk_node rclcpp putm_vcl_interfaces)

add_executable(amk_rx_bridge src/amk_bridge/amk_rx_bridge.cpp)
target_include_directories(
  amk_rx_bridge
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(amk_rx_bridge rclcpp putm_vcl_interfaces)

add_executable(amk_tx_bridge src/amk_bridge/amk_tx_bridge.cpp)
target_include_directories(
  amk_tx_bridge
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(amk_tx_bridge rclcpp putm_vcl_interfaces)

add_executable(can_rx_bridge src/can_bridge/can_rx_bridge.cpp)
target_include_directories(
  can_rx_bridge
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(can_rx_bridge rclcpp putm_vcl_interfaces)

add_executable(can_tx_bridge src/can_bridge/can_tx_bridge.cpp)
target_include_directories(
  can_tx_bridge
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(can_tx_bridge rclcpp putm_vcl_interfaces)

add_executable(rtd_node src/rtd_node/rtd_node.cpp)
target_include_directories(
  rtd_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)
ament_target_dependencies(rtd_node rclcpp putm_vcl_interfaces)

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
