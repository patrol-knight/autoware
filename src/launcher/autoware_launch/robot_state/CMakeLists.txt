cmake_minimum_required(VERSION 3.8)
project(robot_state)

find_package(ament_cmake REQUIRED)

install(PROGRAMS robot_state/node.py DESTINATION lib/${PROJECT_NAME} RENAME robot_state_node)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
