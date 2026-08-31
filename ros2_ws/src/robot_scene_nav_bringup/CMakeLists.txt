cmake_minimum_required(VERSION 3.8)
project(robot_scene_nav_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config behavior_trees DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/plain_slam_map_relay.py
  DESTINATION lib/${PROJECT_NAME})
ament_package()
