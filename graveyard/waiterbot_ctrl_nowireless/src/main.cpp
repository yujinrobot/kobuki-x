/*
  Waiter Node

  LICENSE : BSD - https://raw.github.com/yujinrobot/kobuki-x/license/LICENSE

  Author :  Jihoon Lee
  Date   : Dec 2013
 */

#include <ros/ros.h>
#include "waiterbot_ctrl_nowireless/waiter_node.hpp"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "waiter_controller");
  ros::NodeHandle n;
  waiterbot::WaiterIsolated* wi;

  wi = new waiterbot::WaiterIsolated(n);

  wi->spin();

  delete wi;

  return 0; 
}
