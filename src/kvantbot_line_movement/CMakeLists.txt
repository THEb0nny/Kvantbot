cmake_minimum_required(VERSION 3.0.2)
project(kvantbot_line_movement)

find_package(catkin REQUIRED COMPONENTS
  roscpp
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

add_executable(line_sensors src/line_sensors.cpp)
target_link_libraries(line_sensors ${catkin_LIBRARIES} -lGpioExpanderPi -lpthread -lcrypt -lm -lrt)