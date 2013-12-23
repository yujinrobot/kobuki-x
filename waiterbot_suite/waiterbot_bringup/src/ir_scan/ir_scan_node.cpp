/*
 * ir_scan_node.cpp
 *
 *  Created on: May 31, 2012
 *      Author: jorge
 */

#include "waiterbot/ir_scan.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ir_scan");

  ros::NodeHandle nh;

  waiterbot::IrScanNode node;
  if (node.init(nh) == false)
    return -1;

  if (node.spin() == false)
    return -1;

  return 0;
}
