/*
  WaiterIsolated Class

  LICENSE : BSD - https://raw.github.com/yujinrobot/kobuki-x/license/LICENSE

  Author :  Jihoon Lee
  Date   : Dec 2013
 */

#include "waiterbot_ctrl_nowireless/waiter_node.hpp"

namespace waiterbot {

  bool WaiterIsolated::processOrder(const int drink)
  {
    /*
    if(recordOrderOrigin() == false)
    {
      return endDelivery(false);
    }
    */

    ROS_INFO("Go to vending machine");
    // go to vending machine
    if(goToVendingMachine() == false)
    {
      return endDelivery(false);
    }

    /*
    // call vending machine to get a drink 
    if(callVendingMachine() == false)
    {
      return endDelivery(false);
    }

    // wait for response from vending machine
    if(waitForDrink() == false)
    {
      return endDelivery(false);
    }
    */

    // go back to the location where it got drink order
    ROS_INFO("Go to Customer");
    if(servingDrink() == false)
    {
      return endDelivery(false);
    }

    return endDelivery(true);
  }

}
