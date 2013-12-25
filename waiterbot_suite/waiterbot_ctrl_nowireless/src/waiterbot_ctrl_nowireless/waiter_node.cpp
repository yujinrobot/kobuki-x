/*
  WaiterIsolated Class

  LICENSE : BSD - https://raw.github.com/yujinrobot/kobuki-x/license/LICENSE# is used, also find other catkin packages

  Author :  Jihoon Lee
  Date   : Dec 2013
 */

#include "waiterbot_ctrl_nowireless/waiter_node.hpp"

namespace waiterbot {
  WaiterIsolated::WaiterIsolated(ros::NodeHandle& n) : nh_(n) 
  {  
    initialized_ = false;
    waypointsReceived_ = false;
    init();
  }

  WaiterIsolated::~WaiterIsolated() {  }

  void WaiterIsolated::init()
  {
    // listen to green and red buttons
    sub_digital_input_ = nh_.subscribe("digital_input", 5, & WaiterIsolated::digitalInputCB, this);

    // listen to waypoints
    sub_waypoints_ = nh_.subscribe("waypoints", 5, &WaiterIsolated::waypointsCB, this);
  }

  void WaiterIsolated::digitalInputCB(const kobuki_msgs::DigitalInputEvent::ConstPtr& msg)
  {
  }

  void WaiterIsolated::waypointsCB(const yocs_msgs::WaypointList::ConstPtr& msg)
  {
  }

  void WaiterIsolated::spin() {
  }
}
