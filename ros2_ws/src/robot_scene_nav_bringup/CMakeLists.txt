cmake_minimum_required(VERSION 3.8)
project(robot_scene_nav_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config behavior_trees DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/plain_slam_nav_frame_bridge.py scripts/localization_health_monitor.py scripts/observed_floor_map.py
  DESTINATION lib/${PROJECT_NAME})
# 旧的地图 relay 保留在仓库里，但不再由任何 launch 启动：/map 现在由
# plain_slam_nav_frame_bridge.py 独家发布（计划书 §6.2）。
install(PROGRAMS scripts/plain_slam_map_relay.py
  DESTINATION lib/${PROJECT_NAME})
find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(slam_toolbox REQUIRED)
# Humble's Ubuntu 22.04 SLAM binaries use Boost 1.74. A Conda Boost 1.82
# discovered first would mix two serialization/thread ABIs in this process.
if("$ENV{ROS_DISTRO}" STREQUAL "foxy")
  set(go2w_boost_version 1.71.0)
else()
  set(go2w_boost_version 1.74.0)
endif()
find_package(Boost ${go2w_boost_version} EXACT REQUIRED COMPONENTS thread serialization
  PATHS "/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake/Boost-${go2w_boost_version}" NO_DEFAULT_PATH)
find_package(TBB REQUIRED)
add_executable(tf_authority_monitor src/tf_authority_monitor.cpp)
ament_target_dependencies(tf_authority_monitor rclcpp tf2_msgs std_msgs)
# ROS workers use the sourced ROS library path, never a cached Conda RPATH.
set_target_properties(tf_authority_monitor PROPERTIES SKIP_BUILD_RPATH TRUE INSTALL_RPATH "")
install(TARGETS tf_authority_monitor DESTINATION lib/${PROJECT_NAME})
add_executable(keyframe_slam_toolbox src/keyframe_slam_toolbox.cpp)
if("$ENV{ROS_DISTRO}" STREQUAL "foxy")
  # Foxy Debian headers retain a source-tree-relative karto include path.
  # Correct a private build copy; leave /opt/ros and the old deployment intact.
  set(go2w_foxy_headers "${CMAKE_CURRENT_BINARY_DIR}/foxy_include")
  file(COPY "/opt/ros/foxy/include/slam_toolbox" DESTINATION "${go2w_foxy_headers}")
  file(GLOB go2w_slam_headers "${go2w_foxy_headers}/slam_toolbox/*.hpp")
  foreach(go2w_header IN LISTS go2w_slam_headers)
    file(READ "${go2w_header}" go2w_header_text)
    string(REPLACE "../lib/karto_sdk/include/karto_sdk/Mapper.h"
      "karto_sdk/Mapper.h" go2w_header_fixed "${go2w_header_text}")
    if(NOT go2w_header_fixed STREQUAL go2w_header_text)
      file(WRITE "${go2w_header}" "${go2w_header_fixed}")
    endif()
  endforeach()
  target_include_directories(keyframe_slam_toolbox BEFORE PRIVATE "${go2w_foxy_headers}")
  target_compile_definitions(keyframe_slam_toolbox PRIVATE GO2W_FOXY_SLAM_TOOLBOX=1)
endif()
ament_target_dependencies(keyframe_slam_toolbox rclcpp slam_toolbox std_msgs)
# Humble exports both rosidl targets and legacy SLAM libraries. ament's modern
# target preference omits the latter, so link its exported library list too.
set(go2w_slam_libraries)
foreach(library async_slam_toolbox toolbox_common kartoSlamToolbox)
  find_library(go2w_${library} NAMES ${library}
    PATHS "${slam_toolbox_DIR}/../../../lib" NO_DEFAULT_PATH REQUIRED)
  list(APPEND go2w_slam_libraries "${go2w_${library}}")
endforeach()
target_link_libraries(keyframe_slam_toolbox ${go2w_slam_libraries} Boost::thread Boost::serialization TBB::tbb)
set_target_properties(keyframe_slam_toolbox PROPERTIES SKIP_BUILD_RPATH TRUE INSTALL_RPATH "")
install(TARGETS keyframe_slam_toolbox DESTINATION lib/${PROJECT_NAME})
ament_package()
