cmake_minimum_required(VERSION 3.0.2)
project(kvantbot_pdd)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  image_transport
  camera_info_manager
  cv_bridge
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS src/traffic_sign_detect.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(contest src/contest.cpp)
target_link_libraries(contest ${catkin_LIBRARIES})