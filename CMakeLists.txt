cmake_minimum_required(VERSION 3.8)
project(putm_ev_amk_2023)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(putm_ev_amk_2023 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AmkStatus.msg"
  "msg/AmkControl.msg"
  "msg/AmkData.msg"
)

add_executable(amk_node src/amk_node/amk_node.cpp src/amk_node/amk_startup.cpp)
target_include_directories(amk_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(amk_node rclcpp putm_ev_amk_2023)

add_executable(amk_bridge_rx_node src/amk_bridge_node/amk_bridge_rx.cpp)
target_include_directories(amk_bridge_rx_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(amk_bridge_rx_node rclcpp putm_ev_amk_2023)

add_executable(amk_bridge_tx_node src/amk_bridge_node/amk_bridge_tx.cpp)
target_include_directories(amk_bridge_tx_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(amk_bridge_tx_node rclcpp putm_ev_amk_2023)

install(TARGETS
  amk_node
  amk_bridge_rx_node
  amk_bridge_tx_node
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
