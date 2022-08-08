cmake_minimum_required(VERSION 3.0.2)
project(phantomSLAM)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lboost_system -DGTSAM_USE_SYSTEM_EIGEN=ON")
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
  pcl_conversions
  pcl_ros
  message_generation
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  fast_gicp
)

#set(BOOST_LIBRARYDIR "/usr/lib/x86_64-linux-gnu/") ### jw add
#set(Boost_USE_STATIC_LIBS OFF) ### jw add
#set(Boost_USE_MULTITHREADED ON) ### jw add
#set(Boost_USE_STATIC_RUNTIME OFF) ### jw add
#add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY) ### jw add

#find_package(Boost REQUIRED COMPONENTS) ### jw add

find_package(OpenCV)
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)
find_package(GTSAM REQUIRED QUIET)
#find_package(Boost REQUIRED COMPONENTS system timer thread serialization chrono)

add_message_files(
	FILES 
	EspMeasurements.msg
	WheelSpeedMeasurements.msg
	GearState.msg
)

generate_messages(DEPENDENCIES std_msgs)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
# catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES phantomSLAM
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
# )
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
 DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  #${BOOST_INCLUDE_DIR}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/phantomSLAM.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/phantomSLAM_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(preprocess_node src/preprocess.cpp)
target_link_libraries(preprocess_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

add_executable(avm2pointcloud_node src/avm2pointcloud.cpp)
target_link_libraries(avm2pointcloud_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

add_executable(mapping_node src/mapping.cpp)
target_link_libraries(mapping_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

add_executable(comparing_node src/loam_vs_YW.cpp)
target_link_libraries(comparing_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

add_executable(localmapping_node src/localmapping.cpp)
target_compile_options(localmapping_node PRIVATE ${OpenMP_CXX_FLAGS})
target_compile_features(localmapping_node PRIVATE cxx_std_17)
target_link_libraries(localmapping_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenMP_CXX_FLAGS} gtsam)

add_executable(optimization_node src/optimization.cpp)
target_compile_options(optimization_node PRIVATE ${OpenMP_CXX_FLAGS})
target_compile_features(optimization_node PRIVATE cxx_std_17)
target_link_libraries(optimization_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenMP_CXX_FLAGS} gtsam )

