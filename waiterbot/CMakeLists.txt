cmake_minimum_required(VERSION 2.8.3)
project(waiterbot)

## Find catkin macros and libraries
find_package(catkin REQUIRED tf roscpp actionlib actionlib_msgs sensor_msgs move_base_msgs kobuki_msgs cafe_msgs yocs_msgs
                             ar_track_alvar arduino_interface adc_driver yocs_math_toolkit)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES waiterbot
  CATKIN_DEPENDS tf roscpp actionlib actionlib_msgs sensor_msgs move_base_msgs kobuki_msgs cafe_msgs yocs_msgs
                 ar_track_alvar arduino_interface adc_driver yocs_math_toolkit
  DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## Declare the executables
add_executable(ir_scan_node src/ir_scan_node.cpp)

add_executable(waiterbot_node src/waiter_node.cpp
                              src/waiter_tasks.cpp
                              src/waiter_delivery_handles.cpp
                              src/waiter_callbacks.cpp
                              src/ar_markers.cpp
                              src/nav_watchdog.cpp
                              src/navigator.cpp)

## Add cmake target dependencies of the executable/library
add_dependencies(waiterbot_node kobuki_msgs cafe_msgs_gencpp yocs_msgs_gencpp ar_track_alvar_gencpp)

## Specify libraries to link a library or executable target against
target_link_libraries(ir_scan_node
  ${catkin_LIBRARIES}
)

target_link_libraries(waiterbot_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# TODO
