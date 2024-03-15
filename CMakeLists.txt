cmake_minimum_required(VERSION 3.8)
project(putm_pm09_vcl)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(putm_pm09_vcl REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AmkStatus.msg"
  "msg/AmkControl.msg"
  "msg/AmkData.msg"
  "msg/Frontbox.msg"
)

add_executable(amk_node src/amk_node/amk_node.cpp)
target_include_directories(amk_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(amk_node rclcpp putm_pm09_vcl sensor_msgs)

add_executable(amk_bridge_rx_node src/amk_bridge/amk_rx_bridge.cpp)
target_include_directories(amk_bridge_rx_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(amk_bridge_rx_node rclcpp putm_pm09_vcl)

add_executable(amk_bridge_tx_node src/amk_bridge/amk_tx_bridge.cpp)
target_include_directories(amk_bridge_tx_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(amk_bridge_tx_node rclcpp putm_pm09_vcl)

add_executable(can_rx_bridge_node src/can_bridge/can_rx_bridge.cpp)
target_include_directories(can_rx_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(can_rx_bridge_node rclcpp putm_pm09_vcl)

add_executable(can_tx_bridge_node src/can_bridge/can_tx_bridge.cpp)
target_include_directories(can_tx_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(can_tx_bridge_node rclcpp putm_pm09_vcl)

install(TARGETS
  amk_node
  amk_bridge_rx_node
  amk_bridge_tx_node
  can_tx_bridge_node
  can_rx_bridge_node
  DESTINATION lib/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
