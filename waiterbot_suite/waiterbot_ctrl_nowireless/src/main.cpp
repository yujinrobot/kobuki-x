/*
  Waiter Node

  LICENSE : BSD - https://raw.github.com/yujinrobot/kobuki-x/license/LICENSE

  Author :  Jihoon Lee
  Date   : Dec 2013
 */

#include "waiterbot_ctrl_nowireless/waiter_node.hpp"

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "waiter_controller");
  waiterbot::WaiterIsolated* wi;

  wi = new waiterbot::WaiterIsolated();

  wi->spin();

  delete wi;

  return 0; 
}
