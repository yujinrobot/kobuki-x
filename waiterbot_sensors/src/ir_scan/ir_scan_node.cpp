/*
 * ir_scan_node.cpp
 *
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/kobuki-x/license/LICENSE
 *
 *  Created on: May 31, 2012
 *      Author: jorge
 */

#include "waiterbot_sensors/ir_scan.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ir_scan");

  ros::NodeHandle nh;

  waiterbot::IrScanNode node(nh);

  ROS_INFO("IR Scan node : initialized");
  node.spin(); 

  return 0;
}
