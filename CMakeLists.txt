cmake_minimum_required(VERSION 3.0.2)
project(falcon_controller)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
  geometry_msgs
  sensor_msgs
  visualization_msgs
)

add_service_files(
  FILES
  ModeChange.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    geometry_msgs
    sensor_msgs
    visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/test_moveIt.py
  scripts/falcon_workspace_visualization.py
  scripts/panda_moveit.py
  scripts/panda_mapped.py
  scripts/volume_node.py
  scripts/mode_change.py
  scripts/calibration.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
