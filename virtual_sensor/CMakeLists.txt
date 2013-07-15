cmake_minimum_required(VERSION 2.8.3)
project(virtual_sensor)

## Find catkin macros and libraries
find_package(catkin REQUIRED tf roscpp message_generation sensor_msgs geometry_msgs semantic_region_handler math_toolkit)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Wall.msg
  WallList.msg
  Column.msg
  ColumnList.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES virtual_sensor
  CATKIN_DEPENDS tf roscpp message_runtime sensor_msgs geometry_msgs semantic_region_handler math_toolkit
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(virtual_sensor_node src/virtual_sensor_node.cpp)

## Add cmake target dependencies of the executable/library
add_dependencies(virtual_sensor_node virtual_sensor_gencpp semantic_region_handler_gencpp)

## Specify libraries to link a library or executable target against
target_link_libraries(virtual_sensor_node ${catkin_LIBRARIES})

#############
## Install ##
#############

# TODO

#############
## Testing ##
#############
