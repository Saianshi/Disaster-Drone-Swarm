cmake_minimum_required(VERSION 3.8)
project(disaster_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch models worlds rviz description
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  nodes/swarm_controller.py
  nodes/mission_node.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
