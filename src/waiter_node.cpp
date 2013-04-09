/*
 * waiter_node.cpp
 *
 *  Created on: Apr 6, 2013
 *      Author: jorge
 */

#include <tf/tf.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/Led.h>
#include <kobuki_msgs/Sound.h>

#include "waiterbot/waiter_node.hpp"

namespace waiterbot
{

bool WaiterNode::init()
{
  global_markers_.markers.push_back(ar_track_alvar::AlvarMarker());  // TODO do from semantic map!!!!!!!!!!!!!!



  as_.start();

//  deliver_as_  = nh_., name, boost::bind(&WaiterNode::deliverOrderCB, this, _1), false),

  cmd_vel_pub_  = nh_.advertise <geometry_msgs::Twist> ("mobile_base/commands/velocity", 1);
  led_1_pub_    = nh_.advertise <kobuki_msgs::Led>     ("mobile_base/commands/led1", 1);
  led_2_pub_    = nh_.advertise <kobuki_msgs::Led>     ("mobile_base/commands/led2", 1);
  sound_pub_    = nh_.advertise <kobuki_msgs::Sound>   ("mobile_base/commands/sound", 1);

  sensors_sub_  = nh_.subscribe("core_sensors",   5, &WaiterNode::coreSensorsCB, this);
  odometry_sub_ = nh_.subscribe("odometry",       5, &WaiterNode::odometryCB,    this);
  ar_pose_sub_  = nh_.subscribe("ar_pose_marker", 5, &ARMarkers::arPoseMarkerCB, &ar_markers_);

  return true;
}

void WaiterNode::deliveryCB(const cafe_msgs::DeliverOrderGoal::ConstPtr& goal)
{
  order_ = goal->order;
  ROS_INFO("Deliver order action requested [order: %d, table: %d]", order_.order_id, order_.table_id);

//  if (status_ == robot IDLE)
//  {
    boost::thread wakeUpThread(&WaiterNode::wakeUp, this);
//  }
  //order_status_ = cafe_msgs::Status::IDLE;//TODO ojo; mato la thread????

  // publish the feedback
  feedback_.status = cafe_msgs::Status::IDLE;//order_status_;
  as_.publishFeedback(feedback_);

//  if(success)
//  {
//    result_.sequence = feedback_.sequence;
//    ROS_INFO("%s: Succeeded", action_name_.c_str());
//    // set the action state to succeeded
//    as_.setSucceeded(result_);
//  }
}

void WaiterNode::coreSensorsCB(const kobuki_msgs::SensorState::ConstPtr& msg)
{
  core_sensors_ = *msg;
}

void WaiterNode::odometryCB(const nav_msgs::Odometry::ConstPtr& msg)
{
  odometry_ = *msg;
}

void WaiterNode::wakeUp()
{
  ROS_DEBUG("Waking up! first try to recognize our own nest; slowly moving back...");

  // move back until we detect the AR marker identifying this robot's docking station
  geometry_msgs::Twist vel;
  vel.linear.x = - 0.1;
  cmd_vel_pub_.publish(vel);

  bool timeout = false;
  ros::Time t0 = ros::Time::now();
  ar_track_alvar::AlvarMarkers spotted_markers;
  while ((ar_markers_.spotted(0.1, ar_track_alvar::AlvarMarkers(), global_markers_, spotted_markers) == false) &&
         (timeout == false))
  {
    ros::Duration(0.1).sleep();
    if ((ros::Time::now() - t0).toSec() >= SPOT_BASE_MARKER_TIMEOUT)
    {
      timeout = true;
    }
  }

  // stop after moving a bit more
  ros::Duration(1.0).sleep();
  vel.linear.x = 0.0;
  cmd_vel_pub_.publish(vel);


  if (timeout == true)
  {
    ROS_WARN("Unable to detect docking station AR marker; aborting wake up!");
    return;

    // TODO do nothing more by now, but we will want to make an error sound, red leds, etc.
  }

  ar_track_alvar::AlvarMarker closest_marker;
  ar_markers_.closest(spotted_markers, global_markers_, closest_marker);
  ROS_DEBUG("Docking station AR marker %d spotted! Look for a global marker to find where I am...", closest_marker.id);




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

  ros::spin();

  return 0;
}

