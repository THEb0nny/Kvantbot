cmake_minimum_required(VERSION 3.0.2)
project(kvantbot_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
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

#catkin_install_python(PROGRAMS src/buzzer_module.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

add_executable(encoders src/encoders.cpp)
target_link_libraries(encoders ${catkin_LIBRARIES} -lwiringPi -lpthread -lcrypt -lm -lrt -Wno-psabi)

add_executable(dc_motors src/dc_motors.cpp)
#target_link_libraries(dc_motors ${catkin_LIBRARIES} -lwiringPi -lGpioExpanderPi -lpthread -lcrypt -lm -lrt)
target_link_libraries(dc_motors ${catkin_LIBRARIES} -lwiringPi -lpthread -lcrypt -lm -lrt)

add_executable(line_sensors src/line_sensors.cpp)
target_link_libraries(line_sensors ${catkin_LIBRARIES} -lGpioExpanderPi)

add_executable(btn_module src/button_module.cpp)
target_link_libraries(btn_module ${catkin_LIBRARIES} -lGpioExpanderPi)

add_executable(buzzer_module src/buzzer_module.cpp)
target_link_libraries(buzzer_module ${catkin_LIBRARIES} -lGpioExpanderPi -lwiringPi -lpthread -lrt)