###############################################################################
# Sources
###############################################################################

file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

###############################################################################
# Build
###############################################################################

add_library(waiterbot_ar_marker_tracking ${SOURCES})
add_dependencies(waiterbot_ar_marker_tracking ar_track_alvar_gencpp) # neccessary?
target_link_libraries(waiterbot_ar_marker_tracking yaml-cpp ${catkin_LIBRARIES})

###############################################################################
# Install
###############################################################################
