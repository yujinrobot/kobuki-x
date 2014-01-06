/*
  WaiterIsolated Class

  LICENSE : BSD - https://raw.github.com/yujinrobot/kobuki-x/license/LICENSE

  Author :  Jihoon Lee
  Date   : Dec 2013
 */

#include "waiterbot_ctrl_nowireless/waiter_node.hpp"

namespace waiterbot {

  bool WaiterIsolated::processCommand(const int command)
  {
    int feedback;
    std::string message; 

    switch(command) {
      case waiterbot_msgs::NavCtrlGoTo::GO_TO_VM:
        goToVMCommand(feedback, message);
        break;
      case waiterbot_msgs::NavCtrlGoTo::GO_TO_ORIGIN:
        goToOriginCommand(feedback, message);
        break;
      default:
        std::stringstream sstm;
        sstm <<  "Unknown Command!!!!!! [" <<  feedback << "]";
        message = sstm.str();
        feedback = -1;
    }

    return endCommand(feedback, message);
  }

  void WaiterIsolated::goToVMCommand(int& feedback, std::string& message)
  {
    std::string local_message = "";
    // record the robot location
//    if(recordOrderOrigin(local_message) == false)
//    {
//      message = "Error while recording origin. Reason : " + local_message;
//      feedback = waiterbot_msgs::NavCtrlStatus::ERROR;
//      return;
//    }

    if(goToVendingMachine(message) == false) 
    {
      if(cancel_order_) 
      {
        message = "Order has been cancelled";
        feedback = waiterbot_msgs::NavCtrlStatus::CANCEL;

      }
      else 
      {
        message = "Error while going to VM. Reason : " + local_message;
        feedback = waiterbot_msgs::NavCtrlStatus::ERROR;
      }
      return;
    }

    // go to vm with navigation
    feedback = waiterbot_msgs::NavCtrlStatus::VM_ARRIVAL;
    message = "";
  }

  void WaiterIsolated::goToOriginCommand(int& feedback, std::string& message)
  {
    std::string local_message;
    // TODO:  check if robot location is recorded, if not spit error

    // go to origin of order
    if(goToOrigin(local_message) == false) 
    {
      if(cancel_order_) 
      {
        message = "Order has been cancelled";
        feedback = waiterbot_msgs::NavCtrlStatus::CANCEL;
      }
      else if(tray_empty_) 
      {
        message = "Drink has been picked up while delivery...";
        feedback = waiterbot_msgs::NavCtrlStatus::ORIGIN_ARRIVAL;
      }
      else 
      {
        message = "Error while going to origin. Reason : " + local_message;
        feedback = waiterbot_msgs::NavCtrlStatus::ERROR;
      }
      return;
    }

    feedback = waiterbot_msgs::NavCtrlStatus::ORIGIN_ARRIVAL;
    message = "";
  }
}
