cmake_minimum_required(VERSION 3.0.2)
project(kvantbot_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kvantbot_driver
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(encoders src/encoders.cpp)
target_link_libraries(encoders ${catkin_LIBRARIES} -lwiringPi -lpthread -lcrypt -lm -lrt -Wno-psabi)

add_executable(dc_motors src/dc_motors.cpp)
#target_link_libraries(dc_motors ${catkin_LIBRARIES} -lwiringPi -lGpioExpanderPi -lpthread -lcrypt -lm -lrt)
target_link_libraries(dc_motors ${catkin_LIBRARIES} -lwiringPi -lGpioExpanderPi -lpthread -lcrypt -lm -lrt)