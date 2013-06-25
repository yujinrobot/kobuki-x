/*
 * waiter_callbacks.cpp
 *
 *  Created on: Apr 6, 2013
 *      Author: jorge
 */

#include <visualization_msgs/MarkerArray.h>

#include "waiterbot/waiter_node.hpp"

namespace waiterbot
{

void WaiterNode::tablePosesCB(const semantic_region_handler::TablePoseList::ConstPtr& msg)
{
  // Just take first message; ignore the rest, as global markers list is not dynamic
  if ((table_poses_.tables.size() == 0) && (msg->tables.size() > 0))
  {
    // We will also publish table markers to help visualizing single robot navigation in a rocon environment
    visualization_msgs::MarkerArray markers_array;

    table_poses_ = *msg;
    ROS_INFO("%lu table pose(s) received", table_poses_.tables.size());
    for (unsigned int i = 0; i < table_poses_.tables.size(); i++)
    {
      visualization_msgs::Marker marker;
      marker.header.frame_id = global_frame_;
      marker.header.stamp = ros::Time::now();
      marker.ns = table_poses_.tables[i].name;
      marker.id = i;
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.action = visualization_msgs::Marker::ADD;
      marker.scale.x = table_poses_.tables[i].radius * 2.0;
      marker.scale.y = table_poses_.tables[i].radius * 2.0;
      marker.scale.z = 0.1;
      marker.pose = table_poses_.tables[i].pose_cov_stamped.pose.pose;
      marker.color.r = 0.0f;
      marker.color.g = 0.0f;
      marker.color.b = 1.0f;
      marker.color.a = 0.5f;
      markers_array.markers.push_back(marker);

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

    initialized_table_ = true;

    // Is a latched topic, so we just need to publish once
    if (markers_array.markers.size() > 0)
      table_marker_pub_.publish(markers_array);
  }
}

void WaiterNode::digitalInputCB(const kobuki_msgs::DigitalInputEvent::ConstPtr& msg)
{
  // TODO msg->values[1] red button -> use to stop action in course;
  // by now just close the current action with failure, so we can accept new orders; but we cannot cancel current task
  if ((msg->values[1] == false) && (order_.status == cafe_msgs::Status::ERROR))
  {
    // Return the result to Task Coordinator
    ROS_INFO("This is embarrassing... looks like I have being rescued by a human...   %d", as_.isActive());
    cafe_msgs::DeliverOrderResult result;
    result.result = "Robot manually recovered";
    as_.setAborted(result);
    order_.status = cafe_msgs::Status::IDLE;

    // assume robot is not localized
    initialized_ = false;
  }

  if (msg->values[0] == false)
    wait_for_button_ = false;
}

void WaiterNode::coreSensorsCB(const kobuki_msgs::SensorState::ConstPtr& msg)
{
  core_sensors_ = *msg;
}

void WaiterNode::deliverOrderCB()
{
  if (order_.status != cafe_msgs::Status::IDLE)
  {
    ROS_WARN("Waiterbot not in idle status; cannot attend request (current status is %s)", toCStr(order_.status));
//    return;
    // TODO how the hell I inform of this to the task coordinator???
  }

  // Sccept the new goal
  order_ = as_.acceptNewGoal()->order;
  ROS_INFO("Deliver order action requested [order: %d, table: %d]", order_.order_id, order_.table_id);

  //< DEBUG  Fake orders for evaluating individual tasks
  if ((debug_mode_ == true) && (order_.order_id < 0))
  {
    fakeOrderForEasyDebugging(order_.order_id * -1, order_.table_id);
    // Return the result to Task Coordinator
    cafe_msgs::DeliverOrderResult result;
    result.result = "VAMONOS!!!!";
    as_.setSucceeded(result);
    return;
  }
  //> DEBUG

  // starts a thread to process order
  order_process_thread_ = boost::thread(&WaiterNode::processOrder, this, order_);
}

void WaiterNode::preemptOrderCB()
{
  ROS_WARN("Current order preempted [order: %d, table: %d] (current status is %s)",
           order_.order_id, order_.table_id, toCStr(order_.status));
  // set the action state to preempted
  //  TODO WE REALLY WANT???  as_.setPreempted();
  as_.setPreempted();
}


//< DEBUG
void WaiterNode::fakeOrderForEasyDebugging(int order_id, int table_id)
{
  ROS_INFO("FAKE delivery order action requested [order: %d, table: %d]", order_id, table_id);

  if (order_id == 11)       ar_markers_.setTrackerFreq(5);
  if (order_id == 22)       ar_markers_.setTrackerFreq(10);

  if (order_id == 33)       ar_markers_.setTrackerFreq(0);
//  return;
//  if (order_id == 1)       order_.status = cafe_msgs::Status::ERROR;
//  if (order_id == 2)       order_.status = cafe_msgs::Status::WAITING_FOR_KITCHEN;
//  if (order_id == 3)       order_.status = cafe_msgs::Status::WAITING_FOR_USER_CONFIRMATION;
//  if (order_id == 4)       order_.status = cafe_msgs::Status::IDLE;

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
  if (order_id == 13)      ar_markers_.setTrackerFreq(11);
  if (order_id == 14)      ar_markers_.setTrackerFreq(3);
}
//> DEBUG


} // namespace waiterbot
