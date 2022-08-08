cmake_minimum_required(VERSION 2.8.3)
project(undistorted_top_view)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# set(OpenCV_DIR "/opt/ros/kinetic/share")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  cv_bridge
  message_generation
  pcl_conversions
)
# find_package(OpenCV)
FIND_PACKAGE( OpenCV REQUIRED )

## System dependencies are found with CMake's conventions

# set(BOOST_LIBRARYDIR "/usr/lib/x86_64-linux-gnu/") ### jw add
# set(Boost_USE_STATIC_LIBS OFF) ### jw add
# set(Boost_USE_MULTITHREADED ON) ### jw add
# set(Boost_USE_STATIC_RUNTIME OFF) ### jw add
# add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY) ### jw add

find_package(Boost REQUIRED COMPONENTS) ### jw add


## Generate messages in the 'msg' folder
add_message_files(
#	DIRECTORY msg
	FILES
#	PhantomVisionLane.msg	
#	PhantomVisionNetMsg.msg
#	PhantomVisionObject.msg
#	
#	VisionPhantomnetData.msg
#	VisionPhantomnetDataList.msg
#	VisionPhantomnetDetection.msg
#	VisionPhantomnetObject.msg
#	VisionPhantomnetLaneBlob.msg
#	VisionPhantomnetHorizon.msg
#	VisionPhantomnetObjectPart.msg
	ParkingPhantomnetData.msg
	ParkingPhantomnetDetection.msg
	Bounding_Box.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node 
    src/undistorted_top_view.cpp
    src/ocam_functions.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
