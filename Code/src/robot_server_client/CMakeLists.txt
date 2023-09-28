cmake_minimum_required(VERSION 3.8)
project(robot_server_client)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(action_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

add_library(action_client SHARED
  src/robot_action_client.cpp)
target_include_directories(action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_client
  PRIVATE "ROBOT_SERVER_CLIENT_BUILDING_DLL")
ament_target_dependencies(action_client
  "action_interface"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(action_client PLUGIN "robot_server_client::RobotActionClient" EXECUTABLE robot_action_client)
install(TARGETS
  action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

add_library(action_server SHARED
  src/robot_action_server.cpp)



target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
add_library(driver STATIC
include/serial_interface/driver.cpp
include/serial_interface/driver.hpp
include/serial_interface/driver.hpp
include/serial_interface/joints.cpp
include/serial_interface/serial_port.cpp
include/serial_interface/serial_port.hpp
include/serial_interface/postures.hpp
)
target_include_directories(driver PRIVATE include/serial_port)
target_compile_definitions(action_server
  PRIVATE "ROBOT_SERVER_CLIENT_BUILDING_DLL")
ament_target_dependencies(action_server
  "action_interface"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(action_server PLUGIN "robot_server_client::RobotActionServer" EXECUTABLE robot_action_server)
install(TARGETS
  action_server
  driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
