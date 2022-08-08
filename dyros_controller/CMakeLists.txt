cmake_minimum_required(VERSION 2.8.3)
project(dyros_controller)

find_package(catkin REQUIRED COMPONENTS
             roscpp
             std_msgs
             cmake_modules
             message_generation
             pcl_ros
            )
#####################################PCL###########################################
find_package(PCL REQUIRED)
find_package(Eigen REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS system_lib
)
 
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIR}
)

set(HDR_FILES inc/LocalPlannerThread.h )
set(SRC_FILES src/LocalPlannerThread.cpp )

add_executable(dyros_controller ${SRC_FILES} ${HDR_FILES})

target_link_libraries(dyros_controller
                      ${catkin_LIBRARIES}
                      ${Eigen_LIBRARIES}
                      ${PYTHON_LIBRARIES}
                      ${PCL_LIBRARIES}
                     )


