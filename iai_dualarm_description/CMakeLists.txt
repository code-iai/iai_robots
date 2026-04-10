cmake_minimum_required(VERSION 3.10)
project(iai_dualarm_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config robots
  DESTINATION share/${PROJECT_NAME})

ament_package()