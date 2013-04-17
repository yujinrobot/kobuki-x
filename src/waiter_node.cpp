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
  // register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&WaiterNode::deliverOrderCB, this));
  as_.registerPreemptCallback(boost::bind(&WaiterNode::preemptOrderCB, this));
  as_.start();

//  deliver_as_  = nh_., name, boost::bind(&WaiterNode::deliverOrderCB, this, _1), false),

  cmd_vel_pub_  = nh_.advertise <geometry_msgs::Twist> ("mobile_base/commands/velocity", 1);
  led_1_pub_    = nh_.advertise <kobuki_msgs::Led>     ("mobile_base/commands/led1", 1);
  led_2_pub_    = nh_.advertise <kobuki_msgs::Led>     ("mobile_base/commands/led2", 1);
  sound_pub_    = nh_.advertise <kobuki_msgs::Sound>   ("mobile_base/commands/sound", 1);

  sensors_sub_  = nh_.subscribe("core_sensors",   5, &WaiterNode::coreSensorsCB, this);
  odometry_sub_ = nh_.subscribe("odometry",       5, &WaiterNode::odometryCB,    this);

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

  ar_markers_.setRobotPoseCB(boost::bind(&NavWatchdog::arMarkerMsgCB, &nav_watchd_, _1));
  ar_markers_.baseSpottedCB(boost::bind(&Navigator::baseSpottedMsgCB, &navigator_, _1, _2));

  return true;
}

void WaiterNode::deliverOrderCB()
{
  //  if (status_ == busy o jodido )   reject

  // accept the new goal
  order_ = as_.acceptNewGoal()->order;
  ROS_INFO("Deliver order action requested [order: %d, table: %d]", order_.order_id, order_.table_id);

  if (order_.table_id == 0)
//  if (status_ == robot IDLE)
//  {
    boost::thread wakeUpThread(&WaiterNode::wakeUp, this);
//  }
  //order_status_ = cafe_msgs::Status::IDLE;//TODO ojo; mato la thread????

  if (order_.table_id == 1)
    boost::thread dockingThread(&Navigator::dockInBase, &navigator_, ar_markers_.getDockingBasePose());

  if (order_.table_id == 10)
  {

  }


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
void WaiterNode::preemptOrderCB()
{
  ROS_WARN("Current order preempted [order: %d, table: %d]", order_.order_id, order_.table_id);
  // set the action state to preempted
  //  TODO WE REALLY WANT???  as_.setPreempted();
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

  if (ar_markers_.enableTracker() == false)
  {
    ROS_ERROR("Unable to start AR markers tracker; aborting wake up!");
    return;
  }

  // Move back until we detect the AR marker identifying this robot's docking station
  geometry_msgs::Twist vel;
  vel.linear.x = - 0.1;

  bool timeout = false;
  ros::Time t0 = ros::Time::now();
  ar_track_alvar::AlvarMarkers spotted_markers;
  while ((ar_markers_.spotted(1.0, 10, true, spotted_markers) == false) && (timeout == false))
  {
    cmd_vel_pub_.publish(vel);
    ros::Duration(0.1).sleep();
    if ((ros::Time::now() - t0).toSec() >= SPOT_BASE_MARKER_TIMEOUT)
    {
      timeout = true;
    }
  }

  // stop
  vel.linear.x = 0.0;
  cmd_vel_pub_.publish(vel);


  if (timeout == true)
  {
    ROS_WARN("Unable to detect docking station AR marker; aborting wake up!");
    return;

    // TODO do nothing more by now, but we will want to make an error sound, red leds, etc.
  }

  ar_track_alvar::AlvarMarker closest_marker;
  ar_markers_.closest(1.0, 10, true, closest_marker);
  ROS_DEBUG("Docking station AR marker %d spotted! Look for a global marker to find where I am...", closest_marker.id);

  uint32_t base_marker_id = closest_marker.id;
  //base_marker_.header.frame_id = "map";

  // Now look for a global marker to initialize our localization; full spin clockwise
  vel.angular.z = -0.5;
  cmd_vel_pub_.publish(vel);
  ros::Duration(0.2).sleep();

  while (tf::getYaw(odometry_.pose.pose.orientation) <= 0.0)
  {
//if(nav_watchd_.localized()){cmd_vel_pub_.publish(geometry_msgs::Twist());return;}
    cmd_vel_pub_.publish(vel);
    ros::Duration(0.1).sleep();
  //  ROS_ERROR("%f   %d", tf::getYaw(odometry_.pose.pose.orientation), nav_watchd_.localized());
  }

  while (tf::getYaw(odometry_.pose.pose.orientation) >  0.0)
  {
    cmd_vel_pub_.publish(vel);
    ros::Duration(0.1).sleep();
//    ROS_ERROR("%f", tf::getYaw(odometry_.pose.pose.orientation));
  }

  // After a full spin, we should be localized and in front of our docking base marker
  if (nav_watchd_.localized() == false)
  {
    // Something went wrong... we should have a fall-back mechanism, but this is TODO
    ROS_WARN("Still not localized! Probably we cannot see a globally localized marker");
    return;
  }

  ROS_DEBUG("We are now localized; look again for our docking station marker...");

  // Look (again) for our docking station marker; should be just in front of us!
  timeout = false;
  t0 = ros::Time::now();
  while ((ar_markers_.spotDockMarker(base_marker_id) == false) && (timeout == false))
  {
    cmd_vel_pub_.publish(vel);
    ros::Duration(0.1).sleep();
    if ((ros::Time::now() - t0).toSec() >= SPOT_BASE_MARKER_TIMEOUT)
    {
      timeout = true;
    }
  }

  // stop
  vel.angular.z = 0.0;
  cmd_vel_pub_.publish(vel);


  if (timeout == true)
  {
    // Nope! again something went wrong... and again the fall-back mechanism is TODO
    ROS_WARN("Unable to detect docking station AR marker; aborting wake up!");
    return;

    // TODO do nothing more by now, but we will want to make an error sound, red leds, etc.
  }

  ROS_DEBUG("Docking station AR marker %d spotted and registered as a global marker", base_marker_id);


  if (ar_markers_.disableTracker() == false)
  {
    ROS_WARN("Unable to stop AR markers tracker; we are spilling a lot of CPU!");
    return;
  }

  // Now... we are ready to go!   -> go to kitchen

  //chijon  ->  plot MARKERS global

  /*
  vel.angular.z = 0.0;
  cmd_vel_pub_.publish(vel);

  timeout = false;
  t0 = ros::Time::now();
  while ((ar_markers_.spotted(1.0, global_markers_, ar_track_alvar::AlvarMarkers(), spotted_markers) == false) &&
         (timeout == false))
  {
    ros::Duration(0.1).sleep();
    if ((ros::Time::now() - t0).toSec() >= SPOT_POSE_MARKER_TIMEOUT)
    {
      timeout = true;
    }
  }

  // stop after moving a bit more (TODO move until we have the marker in front)
  ros::Duration(1.0).sleep();
  vel.angular.z = 0.0;
  cmd_vel_pub_.publish(vel);

  if (timeout == true)
  {
    ROS_WARN("Unable to detect globally localized AR marker; aborting wake up!");
    return;

    // TODO do nothing more by now, but we will want to make an error sound, red leds, etc.
  }

  ar_markers_.closest(spotted_markers, ar_track_alvar::AlvarMarkers(), closest_marker);
  ROS_DEBUG("Global localization AR marker %d spotted! Initialize amcl...", closest_marker.id);


  // Check the localization watchdog; we should be localized now

  // Complete a loop looking again for OUR docking base marker; it should be more or less in front
  vel.angular.z = -0.5;
  cmd_vel_pub_.publish(vel);
//  ros::Duration(5.0).sleep();
  while (tf::getYaw(odometry_.pose.pose.orientation) > 0.0)  ros::Duration(0.1).sleep();
  while (tf::getYaw(odometry_.pose.pose.orientation) < 0.0)  ros::Duration(0.1).sleep();

  vel.angular.z = 0.0;
  cmd_vel_pub_.publish(vel);
*/
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

