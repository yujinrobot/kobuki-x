cmake_minimum_required(VERSION 2.8.3)
project(waiterbot_ctrl_cafe)

## Find catkin macros and libraries
find_package(catkin REQUIRED tf roscpp actionlib actionlib_msgs sensor_msgs move_base_msgs kobuki_msgs cafe_msgs yocs_msgs
                             ar_track_alvar yocs_ar_marker_tracking yocs_math_toolkit)


## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES waiterbot_ctrl_cafe
  CATKIN_DEPENDS tf roscpp actionlib actionlib_msgs sensor_msgs move_base_msgs kobuki_msgs cafe_msgs yocs_msgs
                 ar_track_alvar yocs_math_toolkit yocs_ar_marker_tracking
  DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(waiterbot_ctrl_cafe_node src/waiter_node.cpp
                                        src/waiter_tasks.cpp
                                        src/waiter_delivery_handles.cpp
                                        src/waiter_callbacks.cpp
                                        src/ar_markers.cpp
                                        src/nav_watchdog.cpp
                                        src/navigator.cpp)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(waiterbot_ctrl_cafe_node kobuki_msgs cafe_msgs_gencpp yocs_msgs_gencpp ar_track_alvar_gencpp)

## Specify libraries to link a library or executable target against
target_link_libraries(waiterbot_ctrl_cafe_node ${catkin_LIBRARIES})

#############
## Install ##
#############

# TODO
