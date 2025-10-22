cmake_minimum_required(VERSION 3.0.2)
project(falcon_controller)

## Dependencies
find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs sensor_msgs visualization_msgs
)

## Include dirs
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Install Python nodes (list scripts explicitly)
catkin_install_python(PROGRAMS
  scripts/test_moveIt.py
  scripts/falcon_visual.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
