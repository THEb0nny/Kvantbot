cmake_minimum_required(VERSION 3.0.2)
project(kvantbot_teleop)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  joy
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(kvantbot_teleop src/kvantbot_teleop.cpp)
target_link_libraries(kvantbot_teleop ${catkin_LIBRARIES})