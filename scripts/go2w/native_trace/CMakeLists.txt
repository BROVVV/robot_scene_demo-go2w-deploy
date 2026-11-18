cmake_minimum_required(VERSION 3.8)
project(go2w_passive_trace)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
add_executable(passive_trace passive_trace.cpp)
target_compile_features(passive_trace PRIVATE cxx_std_17)
set(GO2W_TRACE_JSON_INCLUDE
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../external/unitree_ros2/example/src/include"
  CACHE PATH "Directory containing the bundled nlohmann/json.hpp")
target_include_directories(passive_trace PRIVATE "${GO2W_TRACE_JSON_INCLUDE}")
ament_target_dependencies(passive_trace rclcpp std_msgs nav_msgs)
install(TARGETS passive_trace RUNTIME DESTINATION bin)
