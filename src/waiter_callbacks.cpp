/*
 * waiter_callbacks.cpp
 *
 *  Created on: Apr 6, 2013
 *      Author: jorge
 */

#include "waiterbot/waiter_node.hpp"

namespace waiterbot
{

void WaiterNode::tablePosesCB(const semantic_region_handler::TablePoseList::ConstPtr& msg)
{
  // Just take first message; ignore the rest, as global markers list is not dynamic
  if ((table_poses_.tables.size() == 0) && (msg->tables.size() > 0))
  {
    table_poses_ = *msg;
    ROS_INFO("%lu table pose(s) received", table_poses_.tables.size());
    for (unsigned int i = 0; i < table_poses_.tables.size(); i++)
    {
      // Look for the pickup point
      if (table_poses_.tables[i].name.find("pickup") != std::string::npos)
      {
        ROS_DEBUG("Pickup point: %s", tk::pose2str(table_poses_.tables[i].pose_cov_stamped.pose.pose));
        pickup_pose_.header = table_poses_.tables[i].pose_cov_stamped.header;
        pickup_pose_.pose = table_poses_.tables[i].pose_cov_stamped.pose.pose;
      }
      else
      {
        ROS_DEBUG("%s. rad: %f, pose: %s", table_poses_.tables[i].name.c_str(), table_poses_.tables[i].radius,
                  tk::pose2str(table_poses_.tables[i].pose_cov_stamped.pose.pose));
      }
    }
  }
  initialized_table_ = true;
}

void WaiterNode::digitalInputCB(const kobuki_msgs::DigitalInputEvent::ConstPtr& msg)
{
  if (msg->values[0] == false)
    wait_for_button_ = false;
}

void WaiterNode::coreSensorsCB(const kobuki_msgs::SensorState::ConstPtr& msg)
{
  core_sensors_ = *msg;
}

void WaiterNode::deliverOrderCB()
{
  // accept the new goal
  cafe_msgs::Order order = as_.acceptNewGoal()->order;
  ROS_INFO("Deliver order action requested [order: %d, table: %d]", order.order_id, order.table_id);

  //< DEBUG
  if ((debug_mode_ == true) && (order.order_id < 0))
  {
    fakeOrderForEasyDebugging(order.order_id * -1, order.table_id);
    // Return the result to Task Coordinator
    cafe_msgs::DeliverOrderResult result;
    result.result = "VAMONOS!!!!";
    as_.setSucceeded(result);
    return;
  }
  //> DEBUG

  // starts a thread to process order
  order_process_thread_ = boost::thread(&WaiterNode::processOrder, this, order);
}

void WaiterNode::preemptOrderCB()
{
  ROS_WARN("Current order preempted [order: %d, table: %d]", order_.order_id, order_.table_id);
  // set the action state to preempted
  //  TODO WE REALLY WANT???  as_.setPreempted();
  as_.setPreempted();
}


//< DEBUG
void WaiterNode::fakeOrderForEasyDebugging(int order_id, int table_id)
{
  ROS_INFO("FAKE delivery order action requested [order: %d, table: %d]", order_id, table_id);

  if (order_id == 1)       boost::thread wakeUpThread(&WaiterNode::wakeUp, this);
  if (order_id == 2)       boost::thread wakeUpThread(&WaiterNode::leaveNest, this);
  if (order_id == 3)       boost::thread dockingThread(boost::bind(&Navigator::dockInBase, &navigator_, ar_markers_.getDockingBasePose()));
  if (order_id == 4)       boost::thread dockingThread(boost::bind(&Navigator::dockInBase, &navigator_));
  if (order_id == 5)       boost::thread pickUpThread(&Navigator::pickUpOrder, &navigator_, pickup_pose_);
  if (order_id == 6)
  {
    bool table_found = false;
    for (unsigned int i = 0; i < table_poses_.tables.size(); i++)
    {
      // Look for the requested table's pose (and get rid of the useless covariance)
      if (table_poses_.tables[i].name.find(tk::nb2str(table_id), strlen("table")) != std::string::npos)
      {
        ROS_DEBUG("Target table %d: rad = %f, pose = %s", table_id, table_poses_.tables[i].radius,
                  tk::pose2str(table_poses_.tables[i].pose_cov_stamped.pose.pose));
        geometry_msgs::PoseStamped table_pose;
        table_pose.header = table_poses_.tables[i].pose_cov_stamped.header;
        table_pose.pose = table_poses_.tables[i].pose_cov_stamped.pose.pose;

        boost::thread pickUpThread(&Navigator::deliverOrder, &navigator_,
                                   table_pose, table_poses_.tables[i].radius);
        table_found = true;
        break;
      }
    }

    if (table_found == false)
    {
      ROS_WARN("Table %d not found! bloody jihoon...  ignoring order", table_id);
    }
  }
  if (order_id == 666)     boost::thread kk(&Navigator::moveBaseReset, &navigator_);
  if (order_id == 8)       boost::thread kk(&Navigator::turn, &navigator_, M_PI*1.5);
  if (order_id == 9)       boost::thread kk(&Navigator::turn, &navigator_, -M_PI*0.5);
  if (order_id == 10)      ar_markers_.enableTracker();
  if (order_id == 11)      ar_markers_.disableTracker();
}
//> DEBUG


} // namespace waiterbot
