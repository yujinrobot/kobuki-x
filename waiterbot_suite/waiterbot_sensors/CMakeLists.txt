cmake_minimum_required(VERSION 2.8.3)
project(waiterbot_sensors)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  adc_driver
  ar_track_alvar
  arduino_interface
  roscpp
  sensor_msgs
  yocs_math_toolkit
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(PkgConfig)
pkg_search_module(REQUIRED yaml_cpp)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES waiterbot_sensors
  CATKIN_DEPENDS adc_driver ar_track_alvar arduino_interface roscpp sensor_msgs yocs_math_toolkit
  DEPENDS Boost yaml-cpp
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${yaml-cpp_INCLUDE_DIRS})

## IR Scan Library
add_library(ir_scan_lib src/ir_scan/ir_scan.cpp)
add_dependencies(ir_scan_lib adc_driver arduino_interface)
target_link_libraries(ir_scan_lib ${catkin_LIBRARIES})

add_executable(ir_scan_node src/ir_scan/ir_scan_node.cpp)
add_dependencies(ir_scan_node ir_scan_lib sensor_msgs_gencpp)
target_link_libraries(ir_scan_node ir_scan_lib)


## AR marker processor
add_library(ar_marker_processor_lib src/ar_marker_processor/ar_marker_processor.cpp
                                    src/ar_marker_processor/tf_handlers.cpp
                                    src/ar_marker_processor/yaml_parser.cpp
                                    src/ar_marker_processor/utils.cpp
           )
add_dependencies(ar_marker_processor_lib ar_track_alvar_gencpp)
target_link_libraries(ar_marker_processor_lib yaml-cpp ${catkin_LIBRARIES})

add_executable(ar_marker_processor src/ar_marker_processor/main.cpp)
add_dependencies(ar_marker_processor ar_marker_processor_lib)
target_link_libraries(ar_marker_processor ar_marker_processor_lib)


#############
## Install ##
#############

# TODO
