cmake_minimum_required(VERSION 2.8.3)
project(waiterbot_ctrl_nowireless)

find_package(catkin REQUIRED COMPONENTS
  waiterbot_bringup roscpp
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES waiterbot_ctrl_nowireless
  CATKIN_DEPENDS waiterbot_bringup roscpp
  DEPENDS Boost
)

###########
## Build ##
###########
include_directories( 
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(waiterbot_ctrl_nowireless_lib  src/${PROJECT_NAME}/waiter_node.cpp)
target_link_libraries(waiterbot_ctrl_nowireless_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES})

## Declare a cpp executable
add_executable(waiter_node src/main.cpp)
add_dependencies(waiter_node waiterbot_ctrl_nowireless_lib)
target_link_libraries(waiter_node waiterbot_ctrl_nowireless_lib)

#############
## Install ##
#############

# TODO
