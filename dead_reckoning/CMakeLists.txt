cmake_minimum_required(VERSION 2.8.3)
project(dead_reckoning)
 
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
  sensor_msgs
  roslib
  cv_bridge
#  OpenCV
)
find_package(Boost) 


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  #${OpenCV_INCLUDE_DIR}
)

add_message_files(
	FILES
	EspMeasurements.msg
	WheelSpeedMeasurements.msg
	GearState.msg
)

generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs roslib message_runtime
#  DEPENDS system_lib
)
add_executable(dead_reckoning src/dead_reckoning.cpp src/main.cpp)

target_link_libraries(
  dead_reckoning
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_library(
  dead_reckoning_lib src/dead_reckoning.cpp
)

