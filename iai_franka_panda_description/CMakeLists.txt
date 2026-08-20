cmake_minimum_required(VERSION 3.10)
project(iai_franka_panda_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY mjcf
  DESTINATION share/${PROJECT_NAME})

ament_package()
