cmake_minimum_required(VERSION 3.8)
project(safe_go2w_control)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_api REQUIRED)
find_package(unitree_go REQUIRED)

set(UNITREE_ROS2_ROOT "$ENV{GO2W_UNITREE_ROOT}")
if(NOT UNITREE_ROS2_ROOT)
  set(UNITREE_ROS2_ROOT "$ENV{HOME}/unitree_ros2")
endif()
set(CONTROL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../..")

add_executable(safe_go2w_ros2_move
  "${CONTROL_ROOT}/src/safe_go2w_ros2_move.cpp"
  "${UNITREE_ROS2_ROOT}/example/src/src/common/ros2_sport_client.cpp"
)
target_compile_features(safe_go2w_ros2_move PRIVATE cxx_std_20)
target_include_directories(safe_go2w_ros2_move PRIVATE
  "${UNITREE_ROS2_ROOT}/example/src/include"
  "${UNITREE_ROS2_ROOT}/example/src/include/common"
  "${UNITREE_ROS2_ROOT}/example/src/include/nlohmann"
)
ament_target_dependencies(safe_go2w_ros2_move rclcpp unitree_api unitree_go)

install(TARGETS safe_go2w_ros2_move DESTINATION lib/${PROJECT_NAME})
ament_package()
