/*
  Waiter Node

  LICENSE : BSD - https://raw.github.com/yujinrobot/kobuki-x/license/LICENSE

  Author :  Jihoon Lee
  Date   : Dec 2013
 */

#ifndef _WAITER_NODE_NO_WIRELESS_HPP_
#define _WAITER_NODE_NO_WIRELESS_HPP_

#include <ros/ros.h>
#include <kobuki_msgs/DigitalInputEvent.h>
#include <yocs_msgs/WaypointList.h>
#include <waiterbot_msgs/DrinkOrder.h>
#include "waiterbot_ctrl_nowireless/navigator.hpp"

namespace waiterbot {
  class WaiterIsolated {
    public: // open to everyone
      WaiterIsolated(ros::NodeHandle& n);
      ~WaiterIsolated();
      void spin();
    protected:  // internal functions
      void init();
      bool isInit();  // check if it has received waypoints 
      void digitalInputCB(const kobuki_msgs::DigitalInputEvent::ConstPtr& msg);
      void waypointsCB(const yocs_msgs::WaypointList::ConstPtr& msg);
      void drinkOrderCB(const waiterbot_msgs::DrinkOrder::ConstPtr& msg);

      void processOrder(const int drink);
    private: // variables
      ros::NodeHandle nh_;
      ros::Subscriber sub_digital_input_;
      ros::Subscriber sub_waypoints_;
      ros::Subscriber sub_drinkorder;

      bool initialized_;
      bool waypointsReceived_;

      Navigator navigator_;

      yocs_msgs::WaypointList waypoints_;

      boost::thread order_process_thread_;

  };
}

#endif // _WAITER_NODE_NO_WIRELESS_HPP_
