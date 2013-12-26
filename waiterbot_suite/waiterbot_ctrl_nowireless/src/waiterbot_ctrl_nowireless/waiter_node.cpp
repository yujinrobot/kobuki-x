/*
  WaiterIsolated Class

  LICENSE : BSD - https://raw.github.com/yujinrobot/kobuki-x/license/LICENSE

  Author :  Jihoon Lee
  Date   : Dec 2013
 */

#include "waiterbot_ctrl_nowireless/waiter_node.hpp"

namespace waiterbot {
  WaiterIsolated::WaiterIsolated(ros::NodeHandle& n) : nh_(n), navigator_(n)
  {  
    initialized_ = false;
    waypointsReceived_ = false;
    init();
  }

  WaiterIsolated::~WaiterIsolated() {  }

  void WaiterIsolated::init()
  {
    // parameters 


    // listen to green and red buttons
    sub_digital_input_ = nh_.subscribe("digital_input", 5, & WaiterIsolated::digitalInputCB, this);

    // listen to waypoints
    sub_waypoints_ = nh_.subscribe("waypoints", 5, &WaiterIsolated::waypointsCB, this);
    // listen to drink order message
    sub_drinkorder = nh_.subscribe("drink_order", 1, &WaiterIsolated::drinkOrderCB, this);

  }

  bool WaiterIsolated::isInit() {
    if(waypointsReceived_ && navigator_.isInit())
      return true;
    else
      return false;
  }

  void WaiterIsolated::digitalInputCB(const kobuki_msgs::DigitalInputEvent::ConstPtr& msg)
  {
    ROS_INFO("In digital input Call back");
  }

  void WaiterIsolated::waypointsCB(const yocs_msgs::WaypointList::ConstPtr& msg)
  {
    waypoints_ = *msg;
    waypointsReceived_ = true;
  }

  void WaiterIsolated::drinkOrderCB(const waiterbot_msgs::DrinkOrder::ConstPtr& msg)
  {
    ROS_INFO("Drink Order : %d", msg->drink);

    // starts to serve.
    order_process_thread_ = boost::thread(&WaiterIsolated::processOrder, this, msg->drink);
  }

  void WaiterIsolated::spin() {
    ros::spin();
  }
}
