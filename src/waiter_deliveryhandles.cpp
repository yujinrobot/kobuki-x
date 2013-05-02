/*
 * waiter_deliveryhandles.cpp
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
   // if(getReadyToWork() == false) { return setFailure("Waiter failed to go to pickup place"); }
    
    // 1. goto pickup place Navigator::pickUpOrder
  //  if(navigator_.pickUpOrder(pickup_pose_) == false) { return setFailure("Waiter failed to go to pickup place");}
    sendFeedback(cafe_msgs::Status::ARRIVE_KITCHEN);
    ros::Duration(1).sleep();
    sendFeedback(cafe_msgs::Status::WAITING_FOR_KITCHEN);
    ros::Duration(1).sleep();
    // 2. Wait for button
    if(waitForButton() == false) { return setFailure("Waiter didn't receive the button from kitchen"); }
    sendFeedback(cafe_msgs::Status::IN_DELIVERY);
    
    // 3. goto table     Navigator::deliverOrder
  //  if(gotoTable(order.table_id) == false) { return setFailure("Waiter failed to go to table"); }
    sendFeedback(cafe_msgs::Status::ARRIVE_TABLE);
    ros::Duration(1).sleep();
    sendFeedback(cafe_msgs::Status::WAITING_FOR_USER_CONFIRMATION);
    ros::Duration(1).sleep();
    // 4. wait for button
    if(waitForButton() == false) { return setFailure("Waiter didn't receive the button from customer"); }
    sendFeedback(cafe_msgs::Status::COMPLETE_DELIVERY);
    ros::Duration(1).sleep();
    sendFeedback(cafe_msgs::Status::RETURNING_TO_DOCK);
    ros::Duration(1).sleep();
 /* 
    // 5. return to dock Navigator::dockInBase
  //  if(navigator_.dockInBase(ar_markers_.getDockingBasePose())) { return setFailure("Waiter failed to go back to nest"); }
  */
    sendFeedback(cafe_msgs::Status::END_DELIVERY_ORDER);
    ros::Duration(1).sleep();
    // Return the result to Task Coordinator
    cafe_msgs::DeliverOrderResult   result;
    result.result = "YA VEREMOS...";
    as_.setSucceeded(result);
    return true;
  }

  bool WaiterNode::gotoTable(int table_id)
  {
    // find table pose
    bool table_found = false;
    float radius;
    geometry_msgs::PoseStamped table_pose;
    for (unsigned int i = 0; i < table_poses_.tables.size(); i++)
    {
      // Look for the requested table's pose (and get rid of the useless covariance)
      if (table_poses_.tables[i].name.find(tk::nb2str(table_id), strlen("table")) != std::string::npos)
      {
        ROS_DEBUG("Target table %d: rad = %f, pose = %s", table_id, table_poses_.tables[i].radius,
                  tk::pose2str(table_poses_.tables[i].pose_cov_stamped.pose.pose));
        table_pose.header = table_poses_.tables[i].pose_cov_stamped.header;
        table_pose.pose = table_poses_.tables[i].pose_cov_stamped.pose.pose;
        radius =  table_poses_.tables[i].radius;
        break;
      }
    }
  
    if (table_found == false)
    {
      ROS_WARN("Table %d not found! bloody jihoon...  ignoring order", table_id);
      return false;
    }
    else {
      return navigator_.deliverOrder(table_pose,radius);
    }
  }

  bool WaiterNode::setFailure(std::string reason)
  {
    cafe_msgs::DeliverOrderResult   result;
    ROS_ERROR_STREAM(reason);
    result.result = reason;
    as_.setSucceeded(result);
    return true;
  }

  void WaiterNode::sendFeedback(int feedback_status)
  {
    cafe_msgs::DeliverOrderFeedback feedback;

    ROS_DEBUG("Sending Feedback %d",feedback_status);
    feedback.status = feedback_status;
    as_.publishFeedback(feedback);
  }

}
