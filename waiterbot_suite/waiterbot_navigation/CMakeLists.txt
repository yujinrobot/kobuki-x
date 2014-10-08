cmake_minimum_required(VERSION 2.8.3)
project(waiterbot_navigation)


find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES waiterbot_navigation
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

catkin_add_env_hooks(25.waiterbot-navigation SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
