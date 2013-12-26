/*
  WaiterIsolated Class

  LICENSE : BSD - https://raw.github.com/yujinrobot/kobuki-x/license/LICENSE

  Author :  Jihoon Lee
  Date   : Dec 2013
 */

#include "waiterbot_ctrl_nowireless/waiter_node.hpp"

namespace waiterbot {

  void WaiterIsolated::processOrder(const int drink)
  {
    // go to vending machine
    if(goToVendingMachine() == false)
    {
      return endDelivery(false);
    }

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

    // go back to the location where it got drink order
    if(servingDrink() == false)
    {
      return endDelivery(false);
    }

    return endDelivery(true);
  }

  bool WaiterIsolated::goToVendingMachine()
  {
    // go to in front of vending machine

    // maybe local navi to located on better place..?
    // run autodock algorithm
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
    return false;
  }

}
