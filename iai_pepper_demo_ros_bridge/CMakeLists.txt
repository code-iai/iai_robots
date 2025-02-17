cmake_minimum_required(VERSION 3.5)
project(iai_pepper_demo_ros_bridge)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

############
# Install  #
#############

install(DIRECTORY src
  DESTINATION share/${PROJECT_NAME})

ament_package()