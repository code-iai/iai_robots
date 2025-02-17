cmake_minimum_required(VERSION 3.5)
project(iai_table_robot_description)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

############
# Install  #
#############

install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY urdf
DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
DESTINATION share/${PROJECT_NAME})

install(DIRECTORY docs
DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME})

install(DIRECTORY robots
DESTINATION share/${PROJECT_NAME})

install(DIRECTORY rosinstall
DESTINATION share/${PROJECT_NAME})

ament_package()