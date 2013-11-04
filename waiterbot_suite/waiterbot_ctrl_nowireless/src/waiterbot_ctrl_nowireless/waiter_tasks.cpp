/*
  WaiterIsolated Class

  LICENSE : BSD - https://raw.github.com/yujinrobot/kobuki-x/license/LICENSE

  Author :  Jihoon Lee
  Date   : Dec 2013
 */

#include "waiterbot_ctrl_nowireless/waiter_node.hpp"

namespace waiterbot {
  bool WaiterIsolated::recordOrderOrigin()
  {
    return true;
  }

  bool WaiterIsolated::goToVendingMachine()
  {
    // go to in front of vending machine
    ROS_INFO("go to %s",loc_vm_.c_str());

    geometry_msgs::PoseStamped vm = map_wp_[loc_vm_];

    vm.header.stamp = ros::Time::now();

    if(navigator_.moveTo(vm) == false) {
      ROS_ERROR("Failed to move");
      return false;
    }
    // maybe local navi to located on better place..?
    // run autodock algorithm
    /*
    if(dockInBase() == false)
    {
      ROS_ERROR("Failed to dock");
      return false;
    }
    */

    return true;
  }

  bool WaiterIsolated::callVendingMachine()
  {
    return false;
  }

  bool WaiterIsolated::waitForDrink()
  {
    return false;
  }

  bool WaiterIsolated::servingDrink()
  {
    // go to in front of vending machine
    ROS_INFO("go to %s",loc_customer_.c_str());
    geometry_msgs::PoseStamped vm = map_wp_[loc_customer_];
    vm.header.stamp = ros::Time::now();

    if(navigator_.moveTo(vm) == false) {
      ROS_ERROR("Failed to move");
      return false;
    }
    return true;
  }

  bool WaiterIsolated::dockInBase() {
    kobuki_msgs::AutoDockingGoal ad_goal;
    ac_autodock_.sendGoal(ad_goal);

    while (ac_autodock_.waitForResult(ros::Duration(3.0)) == false)
    {
      ROS_INFO("Waiting...");
    }


    if(ac_autodock_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Successfully docked...");
      return true;
    }
    else {
      ROS_ERROR("Failed to dock...");
      return false;
    }
  }
}
