cmake_minimum_required(VERSION 3.0.2)
project(zero_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  trajectory_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs trajectory_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/arm_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)