cmake_minimum_required(VERSION 2.8.3)
project(waiterbot_ctrl_nowireless)

find_package(catkin REQUIRED COMPONENTS
  waiterbot_bringup roscpp waiterbot_msgs yocs_msgs kobuki_msgs yocs_math_toolkit tf
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES waiterbot_ctrl_nowireless
  CATKIN_DEPENDS waiterbot_bringup roscpp yocs_msgs kobuki_msgs yocs_math_toolkit tf
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

add_library(waiterbot_tf_handlers src/tf_handlers/tf_handlers.cpp)
target_link_libraries(waiterbot_tf_handlers ${catkin_LIBRARIES})


add_library(waiterbot_navigator src/navigator/navigator.cpp
                                src/navigator/basic_moves.cpp)
add_dependencies(waiterbot_navigator move_base_msgs_gencpp yocs_msgs_gencpp geometry_msgs_gencpp )
target_link_libraries(waiterbot_navigator ${catkin_LIBRARIES})

add_library(waiterbot_nav_watchdog src/nav_watchdog/nav_watchdog.cpp)
add_dependencies(waiterbot_nav_watchdog geometry_msgs_gencpp)
target_link_libraries(waiterbot_nav_watchdog ${catkin_LIBRARIES})


add_library(waiterbot_ctrl_nowireless_lib  src/waiterbot_ctrl_nowireless/waiter_node.cpp
                                           src/waiterbot_ctrl_nowireless/waiter_delivery_handles.cpp  
                                           src/waiterbot_ctrl_nowireless/waiter_tasks.cpp
           )
add_dependencies(waiterbot_ctrl_nowireless_lib waiterbot_msgs_gencpp kobuki_msgs_gencpp yocs_msgs_gencpp waiterbot_navigator)
target_link_libraries(waiterbot_ctrl_nowireless_lib waiterbot_navigator waiterbot_tf_handlers waiterbot_nav_watchdog ${catkin_LIBRARIES} ${Boost_LIBRARIES})



## Declare a cpp executable
add_executable(waiter_node src/main.cpp)
add_dependencies(waiter_node waiterbot_ctrl_nowireless_lib)
target_link_libraries(waiter_node waiterbot_ctrl_nowireless_lib)

#############
## Install ##
#############

# TODO
