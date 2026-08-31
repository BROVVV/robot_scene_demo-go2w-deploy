cmake_minimum_required(VERSION 3.8)
project(go2w_motion_control)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(unitree_api REQUIRED)
find_package(unitree_go REQUIRED)
find_package(go2w_motion_interfaces REQUIRED)

set(UNITREE_ROS2_ROOT "$ENV{GO2W_UNITREE_ROOT}")
if(NOT UNITREE_ROS2_ROOT)
  set(UNITREE_ROS2_ROOT "$ENV{HOME}/unitree_ros2")
endif()

add_library(go2w_motion_core
  src/goal_logger.cpp
  src/leased_sport_client.cpp
  src/motion_state_monitor.cpp
)
target_compile_features(go2w_motion_core PUBLIC cxx_std_20)
target_include_directories(go2w_motion_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  "${UNITREE_ROS2_ROOT}/example/src/include"
)
ament_target_dependencies(go2w_motion_core
  rclcpp std_msgs unitree_api unitree_go go2w_motion_interfaces)

add_executable(go2w_motion_action_server
  src/main.cpp
  src/motion_action_server.cpp
)
target_compile_features(go2w_motion_action_server PRIVATE cxx_std_20)
target_include_directories(go2w_motion_action_server PRIVATE
  include
  "${UNITREE_ROS2_ROOT}/example/src/include"
)
target_link_libraries(go2w_motion_action_server go2w_motion_core)
ament_target_dependencies(go2w_motion_action_server
  rclcpp rclcpp_action std_msgs std_srvs unitree_api unitree_go
  go2w_motion_interfaces)

install(TARGETS go2w_motion_core go2w_motion_action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(test_name IN ITEMS angle_utils goal_validation request_matching yaw_controller)
    ament_add_gtest(test_${test_name} test/test_${test_name}.cpp)
    target_compile_features(test_${test_name} PRIVATE cxx_std_20)
    target_include_directories(test_${test_name} PRIVATE include)
    ament_target_dependencies(test_${test_name} go2w_motion_interfaces)
  endforeach()
endif()

ament_export_include_directories(include)
ament_package()
