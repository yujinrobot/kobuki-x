##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(waiterbot_ar_pair_approach)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

##############################################################################
# Installs
##############################################################################

#install(
#    PROGRAMS 
#        scripts/concert_roles_and_apps
#    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#    )

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
