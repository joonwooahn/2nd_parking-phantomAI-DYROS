cmake_minimum_required(VERSION 2.8.3)
project(phantom_can)
 
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cmake_modules
  message_generation
)
find_package(Eigen REQUIRED)
##################################### QT ##########################################
#find_package(Qt4 REQUIRED COMPONENTS QtCore QtWidgets) ### 
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets) ### jw add

#include(${QT_USE_FILE}) ### jw delete

###################################################################################
catkin_package(
#  INCLUDE_DIRS inc
#  LIBRARIES lib
  CATKIN_DEPENDS roscpp std_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########
# set(BOOST_LIBRARYDIR "/usr/lib/x86_64-linux-gnu/") ### jw add
# set(Boost_USE_STATIC_LIBS OFF) ### jw add
# set(Boost_USE_MULTITHREADED ON) ### jw add
# set(Boost_USE_STATIC_RUNTIME OFF) ### jw add
# add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY) ### jw add

#find_package(Boost REQUIRED COMPONENTS signals) ###
find_package(Boost REQUIRED COMPONENTS) ### jw add

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

## Declare a cpp library
#add_library(ompl_qt_pkg
#   src/${PROJECT_NAME}/ompl_qt_pkg.cpp
#)

######################## test_ui_node #################################

set( HDR_FILES_QT_UI_NODE inc/main.h inc/qnode.h )
set( SRC_FILES_QT_UI_NODE src/main.cpp src/qnode.cpp src/CAN_MCU.cpp src/eCAN.cpp ${MOC_FILES})
set( FORM_FILES_QT_UI_NODE ui/phantom_can_gui.ui)

#QT4_WRAP_CPP(MOC_SRC_H_QT_UI_NODE ${HDR_FILES_QT_UI_NODE}) ### 
#QT4_WRAP_UI(MOC_GUI_H_QT_UI_NODE ${FORM_FILES_QT_UI_NODE}) ### 
QT5_WRAP_CPP(MOC_SRC_H_QT_UI_NODE ${HDR_FILES_QT_UI_NODE}) ### jw add
QT5_WRAP_UI(MOC_GUI_H_QT_UI_NODE ${FORM_FILES_QT_UI_NODE}) ### jw add

## Declare a cpp executable
add_executable(phantom_can ${SRC_FILES_QT_UI_NODE} ${HDR_FILES_QT_UI_NODE} ${MOC_SRC_H_QT_UI_NODE} ${MOC_GUI_H_QT_UI_NODE})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(ompl_qt_pkg_node ompl_qt_pkg_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(phantom_can
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Boost_LIBRARIES}
#eci113DriverLinux
eci113DriverLinux-usb-1.0
#   eci10ADriverLinux
) 
 
qt5_use_modules(phantom_can Widgets) ### jw add
#######################################################################

