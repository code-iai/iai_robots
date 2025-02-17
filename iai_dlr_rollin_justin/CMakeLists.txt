cmake_minimum_required(VERSION 3.5)
project(iai_dlr_rollin_justin)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

############
# Install  #
#############

install(DIRECTORY fbx
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY mjcf
DESTINATION share/${PROJECT_NAME})

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME})


ament_package()