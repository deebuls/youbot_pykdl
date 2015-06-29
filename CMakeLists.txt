cmake_minimum_required(VERSION 2.8.3)
project(youbot_pykdl)

find_package(catkin
  REQUIRED
  COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
  tf
  youbot_core_msgs
  youbot_interface
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  rospy
  geometry_msgs
  sensor_msgs
  urdfdom
  tf
  youbot_interface
  youbot_core_msgs
  DEPENDS
  python_orocos_kdl
  urdfdom
)

install(
  DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
