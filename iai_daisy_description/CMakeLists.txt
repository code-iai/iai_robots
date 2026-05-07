cmake_minimum_required(VERSION 3.10)
project(iai_daisy_description)

find_package(ament_cmake REQUIRED)
find_package(ur_client_library REQUIRED)
find_package(griplink REQUIRED)
install(DIRECTORY launch config robots
  DESTINATION share/${PROJECT_NAME})

ament_package()