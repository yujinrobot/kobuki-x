/*
 * waiter_cbs.cpp
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

  void WaiterNode::coreSensorsCB(const kobuki_msgs::SensorState::ConstPtr& msg)
  {
    core_sensors_ = *msg;
  }

  void WaiterNode::deliverOrderCB()
  {
    // accept the new goal
    cafe_msgs::Order order = as_.acceptNewGoal()->order;
    ROS_INFO("Deliver order action requested [order: %d, table: %d]", order.order_id, order.table_id);
  
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
}
