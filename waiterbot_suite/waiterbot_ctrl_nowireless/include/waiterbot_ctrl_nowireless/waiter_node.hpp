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

namespace waiterbot {
  class WaiterIsolated {
    public: // open to everyone
      WaiterIsolated(ros::NodeHandle& n);
      ~WaiterIsolated();
      void spin();
    protected:  // internal functions
      void init();
      void digitalInputCB(const kobuki_msgs::DigitalInputEvent::ConstPtr& msg);
      void waypointsCB(const yocs_msgs::WaypointList::ConstPtr& msg);
    private: // variables
      ros::NodeHandle nh_;
      ros::Subscriber sub_digital_input_;
      ros::Subscriber sub_waypoints_;

      bool initialized_;
      bool waypointsReceived_;

  };
}

#endif // _WAITER_NODE_NO_WIRELESS_HPP_
