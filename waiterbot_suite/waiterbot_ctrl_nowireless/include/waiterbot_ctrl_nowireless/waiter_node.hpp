/*
  Waiter Node

  LICENSE : BSD - https://raw.github.com/yujinrobot/kobuki-x/license/LICENSE

  Author :  Jihoon Lee
  Date   : Dec 2013
 */

#ifndef _WAITER_NODE_NO_WIRELESS_HPP_
#define _WAITER_NODE_NO_WIRELESS_HPP_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <kobuki_msgs/DigitalInputEvent.h>
#include <yocs_msgs/WaypointList.h>
#include <waiterbot_msgs/DrinkOrder.h>

#include "waiterbot_ctrl_nowireless/navigator.hpp"
#include <kobuki_msgs/AutoDockingAction.h>
#include <geometry_msgs/PoseStamped.h>

#include <map>

namespace waiterbot {

  const std::string DEFAULT_LOC_VM= "loc_vm";
  const std::string DEFAULT_LOC_CUSTOMER = "loc_customer";

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
      bool endDelivery(bool success);

      bool processOrder(const int drink);
        bool recordOrderOrigin();
        bool goToVendingMachine();
        bool callVendingMachine();
        bool waitForDrink();
        bool servingDrink();

      bool dockInBase();
    private: // variables
      ros::NodeHandle nh_;
      ros::Subscriber sub_digital_input_;
      ros::Subscriber sub_waypoints_;
      ros::Subscriber sub_drinkorder;
      actionlib::SimpleActionClient<kobuki_msgs::AutoDockingAction> ac_autodock_;

      std::string loc_vm_;
      std::string loc_customer_;

      bool initialized_;
      bool waypointsReceived_;
      bool inDelivery_;

      Navigator navigator_;

      std::map<std::string, geometry_msgs::PoseStamped> map_wp_;

      boost::thread order_process_thread_;
  };
}

#endif // _WAITER_NODE_NO_WIRELESS_HPP_
