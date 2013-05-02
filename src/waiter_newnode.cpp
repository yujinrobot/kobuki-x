/*
 * waiter_node.cpp
 *
 *  Created on: Apr 6, 2013
 *      Author: jorge
 */

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/Led.h>
#include <kobuki_msgs/Sound.h>

#include "waiterbot/waiter_node.hpp"

namespace waiterbot
{

bool WaiterNode::init()
{
  // register the goal and preempt callbacks
  as_.registerGoalCallback(boost::bind(&WaiterNode::deliverOrderCB, this));
  as_.registerPreemptCallback(boost::bind(&WaiterNode::preemptOrderCB, this));
  as_.start();

  led_1_pub_    = nh_.advertise <kobuki_msgs::Led>     ("mobile_base/commands/led1", 1);
  led_2_pub_    = nh_.advertise <kobuki_msgs::Led>     ("mobile_base/commands/led2", 1);
  sound_pub_    = nh_.advertise <kobuki_msgs::Sound>   ("mobile_base/commands/sound", 1);

  core_sensors_sub_ = nh_.subscribe("core_sensors",    5, &WaiterNode::coreSensorsCB, this);
  table_poses_sub_  = nh_.subscribe("table_pose_list", 1, &WaiterNode::tablePosesCB, this);

  // is robot localized? 
  initialized_ = false;

  // received table pose?
  initialized_table_ = false; 

  // Initialize sub-modules
  if (nav_watchd_.init() == false)
  {
    ROS_ERROR("Navigation watchdog initialization failed; shutting down %s node", node_name_.c_str());
    return false;
  }
  if (ar_markers_.init() == false)
  {
    ROS_ERROR("AR markers initialization failed; shutting down %s node", node_name_.c_str());
    return false;
  }
  if (navigator_.init() == false)
  {
    ROS_ERROR("Navigator initialization failed; shutting down %s node", node_name_.c_str());
    return false;
  }

  // Configure call-backs between submodules
  ar_markers_.setRobotPoseCB(boost::bind(&NavWatchdog::arMarkerMsgCB, &nav_watchd_, _1));
  ar_markers_.baseSpottedCB(boost::bind(&Navigator::baseSpottedMsgCB, &navigator_, _1, _2));

  // Enable safety controller by default
  navigator_.enableSafety();

  return true;
}

void WaiterNode::spin()
{
  ros::Rate rate(50.0);

  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
}

} /* namespace waiterbot */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waiterbot");

  waiterbot::WaiterNode node(ros::this_node::getName());
  if (node.init() == false)
  {
    ROS_ERROR("%s initialization failed", ros::this_node::getName().c_str());
    return -1;
  }
  ROS_INFO("Waiter Initialized");

  node.spin();

  return 0;
}

