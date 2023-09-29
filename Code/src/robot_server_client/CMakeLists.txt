cmake_minimum_required(VERSION 3.8)
project(robot_server_client)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -lboost_unit_test_framework -lboost_system -lboost_filesystem -lboost_program_options -fPIC)
endif()
find_package(ament_cmake REQUIRED)
find_package(action_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Boost REQUIRED)

add_library(driver
  ${CMAKE_CURRENT_SOURCE_DIR}/include/serial_interface/driver.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/serial_interface/joints.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/include/serial_interface/serial_port.cpp
)

target_include_directories(driver PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/serial_interface>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(driver
  Boost::boost
)

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

  target_link_libraries(action_server
  driver)

target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>) 


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
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
ament_package()
