/*
 * ar_marker_processor
 *   main.cpp
 *
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/kobuki-x/license/LICENSE
 *
 *  Created on: Apr 6, 2013
 *      Author: jorge
 *  Modified on: Dec, 2013 
 *      Jihoon Lee
 */

#include "waiterbot_sensors/ar_marker_processor.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ar_marker_processor");
  waiterbot::ARMarkerProcessor arp;
  ROS_INFO("AR Marker Processor : Initialized");
  arp.spin();

  return 0;
}
