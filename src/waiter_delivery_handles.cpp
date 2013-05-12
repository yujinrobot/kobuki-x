/*
 * waiter_delivery_handles.cpp
 *
 *  Created on: May, 2013
 *      Author: Jihoon
 */

#include "waiterbot/waiter_node.hpp"

namespace waiterbot
{

bool WaiterNode::processOrder(cafe_msgs::Order& order)
{
  // wait for semantic pose initialization
  waitForPoses();

  // 0. wakeup or leave nest WaiterNode::wakeUp,leaveNest
  sendFeedback(cafe_msgs::Status::GO_TO_KITCHEN);
  ros::Duration(1).sleep();
  if (getReadyToWork() == false)
  {
    return setFailure("Waiter failed to get ready to work");
  }

  // 1. goto pickup place Navigator::pickUpOrder
  if (navigator_.pickUpOrder(pickup_pose_) == false)
  {
    return setFailure("Waiter failed to go to pickup place");
  }
  sendFeedback(cafe_msgs::Status::ARRIVE_KITCHEN);
  ros::Duration(1).sleep();
  sendFeedback(cafe_msgs::Status::WAITING_FOR_KITCHEN);
  ros::Duration(1).sleep();
  
  // 2. Wait for button
  if (waitForButton() == false)
  {
    return setFailure("Waiter didn't receive the button from kitchen");
  }
  sendFeedback(cafe_msgs::Status::IN_DELIVERY);

  // 3. goto table     Navigator::deliverOrder
  if (gotoTable(order.table_id) == false)
  {
    return setFailure("Waiter failed to go to table");
  }
  sendFeedback(cafe_msgs::Status::ARRIVE_TABLE);
  ros::Duration(1).sleep();
  sendFeedback(cafe_msgs::Status::WAITING_FOR_USER_CONFIRMATION);
  ros::Duration(1).sleep();

  // 4. wait for button
  if (waitForButton() == false)
  {
    return setFailure("Waiter didn't receive the button from customer");
  }
  sendFeedback(cafe_msgs::Status::COMPLETE_DELIVERY);
  ros::Duration(1).sleep();
  sendFeedback(cafe_msgs::Status::RETURNING_TO_DOCK);
  ros::Duration(1).sleep();

  // 5. return to dock Navigator::dockInBase
  if (navigator_.dockInBase(ar_markers_.getDockingBasePose()) == false)
  {
    return setFailure("Waiter failed to go back to nest");
  }

  sendFeedback(cafe_msgs::Status::END_DELIVERY_ORDER);
  ros::Duration(1).sleep();

  return setSucceeded("Delivery successfully completed (hopefully...)");
}

bool WaiterNode::setSucceeded(std::string message)
{
  // Return the result to Task Coordinator
  ROS_INFO_STREAM(message);
  cafe_msgs::DeliverOrderResult result;
  result.result = message;
  as_.setSucceeded(result);
  order_.status = cafe_msgs::Status::IDLE;

  return true;
}

bool WaiterNode::setFailure(std::string message)
{
  // Return the result to Task Coordinator
  ROS_ERROR_STREAM(message);
  cafe_msgs::DeliverOrderResult result;
  result.result = message;
//  as_.setAborted(result);

  // NEW POLITICS:  we don't close the action until we are in the docking base, ready to take a new order, or someone press the red button (manual recovery)

  // Try to go back to nest   TODO  a better feedback would be RECOVERING
  sendFeedback(cafe_msgs::Status::ERROR);

  bool at_base;
  ROS_ERROR("Something went wrong while processing order; try to go back to nest...");
  if (ar_markers_.dockingBaseSpotted() == true)
    at_base = navigator_.dockInBase(ar_markers_.getDockingBasePose());
  else
    at_base = navigator_.dockInBase();

  if (at_base == false)
  {
    ROS_ERROR("Go back to nest failed; we don't have a recovery mechanism, so... please put me on my nest and press the red button to notify TC that I'm ready again");
    order_.status = cafe_msgs::Status::ERROR;
  }
  else
  {
    order_.status = cafe_msgs::Status::IDLE;
    as_.setAborted(result);
  }

  return at_base;
}

void WaiterNode::sendFeedback(int feedback_status)
{
  cafe_msgs::DeliverOrderFeedback feedback;

//  ROS_DEBUG("Sending Feedback %d", feedback_status);
  feedback.status = feedback_status;
  as_.publishFeedback(feedback);

  order_.status = feedback_status;
}

} // namespace waiterbot
