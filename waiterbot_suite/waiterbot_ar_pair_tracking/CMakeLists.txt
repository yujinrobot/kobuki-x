cmake_minimum_required(VERSION 2.8.3)
project(waiterbot_ar_pair_tracking)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  yocs_math_toolkit
  waiterbot_ar_marker_tracking
  ar_track_alvar
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(PkgConfig)
pkg_search_module(REQUIRED yaml_cpp)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES waiterbot_ar_pair_tracking
  CATKIN_DEPENDS roscpp ar_track_alvar geometry_msgs yocs_math_toolkit waiterbot_ar_marker_tracking
  DEPENDS Boost yaml-cpp
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${yaml-cpp_INCLUDE_DIRS})

add_subdirectory(src)


#############
## Install ##
#############

# TODO
