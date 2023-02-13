cmake_minimum_required(VERSION 3.0.2)
project(kvantbot_contest)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kvantbot_line_movement
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS src/traffic_sign_detect.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)