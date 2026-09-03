cmake_minimum_required(VERSION 3.8)
project(go2w_plain_slam_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

# ---------------------------------------------------------------------------
# Core library (header-only logic + shared helpers) used by nodes and tests.
# ---------------------------------------------------------------------------
add_library(go2w_plain_slam_bridge_core INTERFACE)
target_include_directories(go2w_plain_slam_bridge_core INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(go2w_plain_slam_bridge_core INTERFACE
  yaml-cpp)
ament_target_dependencies(go2w_plain_slam_bridge_core INTERFACE
  rclcpp sensor_msgs nav_msgs geometry_msgs diagnostic_msgs std_msgs tf2 tf2_ros)

# ---------------------------------------------------------------------------
# Nodes
# ---------------------------------------------------------------------------
add_executable(pandar_slam_adapter src/pandar_slam_adapter.cpp)
target_link_libraries(pandar_slam_adapter go2w_plain_slam_bridge_core)
ament_target_dependencies(pandar_slam_adapter rclcpp sensor_msgs std_msgs)

add_executable(plain_slam_odom_adapter src/plain_slam_odom_adapter.cpp)
target_link_libraries(plain_slam_odom_adapter go2w_plain_slam_bridge_core)
ament_target_dependencies(plain_slam_odom_adapter rclcpp nav_msgs geometry_msgs)

add_executable(pointcloud_to_occupancy src/pointcloud_to_occupancy.cpp)
target_link_libraries(pointcloud_to_occupancy go2w_plain_slam_bridge_core)
ament_target_dependencies(pointcloud_to_occupancy rclcpp sensor_msgs nav_msgs std_msgs diagnostic_msgs)

add_executable(plain_slam_health_monitor src/plain_slam_health_monitor.cpp)
target_link_libraries(plain_slam_health_monitor go2w_plain_slam_bridge_core)
ament_target_dependencies(plain_slam_health_monitor rclcpp sensor_msgs nav_msgs diagnostic_msgs std_msgs)

install(TARGETS
  pandar_slam_adapter
  plain_slam_odom_adapter
  pointcloud_to_occupancy
  plain_slam_health_monitor
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS scripts/imu_fallback_adapter.py scripts/imu_timestamp_aligner.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(timestamp_policy_test test/timestamp_policy_test.cpp)
  if(TARGET timestamp_policy_test)
    target_link_libraries(timestamp_policy_test go2w_plain_slam_bridge_core)
  endif()

  ament_add_gtest(occupancy_projection_test test/occupancy_projection_test.cpp)
  if(TARGET occupancy_projection_test)
    target_link_libraries(occupancy_projection_test go2w_plain_slam_bridge_core)
  endif()

  ament_add_gtest(transform_utils_test test/transform_utils_test.cpp)
  if(TARGET transform_utils_test)
    target_link_libraries(transform_utils_test go2w_plain_slam_bridge_core)
    target_compile_definitions(transform_utils_test PRIVATE
      TEST_PROJECT_ROOT="${PROJECT_SOURCE_DIR}/../../..")
  endif()
endif()

ament_package()
