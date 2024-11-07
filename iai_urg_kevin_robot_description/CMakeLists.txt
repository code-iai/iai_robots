cmake_minimum_required(VERSION 2.8.3)
project(iai_urg_kevin_robot_description)

find_package(catkin REQUIRED COMPONENTS )

catkin_package()

install(
  DIRECTORY meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
