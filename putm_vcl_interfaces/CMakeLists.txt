cmake_minimum_required(VERSION 3.8)
project(putm_vcl_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Amk/AmkStatus.msg" 
  "msg/Amk/AmkControl.msg" 
  "msg/Amk/AmkData.msg"
  "msg/Frontbox/FrontboxDriverInput.msg"
  "msg/Frontbox/FrontboxData.msg"
  "msg/Dashboard.msg"
  "msg/Rtd.msg"
  "msg/Setpoints.msg"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
