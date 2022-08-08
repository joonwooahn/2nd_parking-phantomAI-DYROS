cmake_minimum_required(VERSION 2.8.3)
project(informed_rrt_star)

 
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cmake_modules
  message_generation
  pcl_ros
)

find_package(Eigen REQUIRED)
#####################################PCL###########################################
ADD_COMPILE_OPTIONS(-std=c++14 )
find_package(PCL REQUIRED QUIET COMPONENTS common)

# find_package(Boost REQUIRED COMPONENTS system serialization)


##################################### OMPL  ##########################################
set(OMPL_LIBRARIES /usr/local/lib/libompl.so)     #MSK
set(OMPL_INCLUDE_DIRS /usr/local/include/ompl-1.5)#MSK
find_package(OMPL)

MESSAGE( STATUS "OMPL_LIBRARY_DIRS: " ${OMPL_LIBRARIES} )
MESSAGE( STATUS "OMPL_INCLUDE_DIRS: " ${OMPL_INCLUDE_DIRS} )
###################################################################################
find_package(Boost REQUIRED) ### jw add

SET(CMAKE_CXX_FLAGS "-std=c++0x")
catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    std_msgs
  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIR}
  ${OMPL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

set( SRC_FILES src/informed_rrt_star.cpp src/biRRTstar.cpp src/RRTstar_target.cpp src/RRTtarget.cpp)
#set( RRT_FILES src/RRTstar.cpp)

file(GLOB STEERING_FUNCTION_RELATED_SRC
    "inc/utiliteis/*.h"
    "inc/*.h"
    "src/hc_related/*.cpp"
    "src/gc_related/*.cpp"
)

add_executable(informed_rrt_star ${SRC_FILES} ${STEERING_FUNCTION_RELATED_SRC})

target_link_libraries(informed_rrt_star
   ${catkin_LIBRARIES}
   ${Eigen_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OMPL_LIBRARIES}
   ${boost_LIBRARIES}
)
#######################################################################
