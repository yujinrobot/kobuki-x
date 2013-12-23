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
#include <visualization_msgs/MarkerArray.h>

#include "waiterbot/waiter_node.hpp"

namespace waiterbot
{

bool WaiterNode::init()
{
  ros::NodeHandle pnh("~");
  pnh.param("debug_mode",   debug_mode_,   false);
  pnh.param("global_frame", global_frame_, std::string("map"));

  // register the goal and preempt callbacks
  as_.registerGoalCallback(boost::bind(&WaiterNode::deliverOrderCB, this));
  as_.registerPreemptCallback(boost::bind(&WaiterNode::preemptOrderCB, this));
  as_.start();

  led_1_pub_ = nh_.advertise <kobuki_msgs::Led>     ("/mobile_base/commands/led1", 1);
  led_2_pub_ = nh_.advertise <kobuki_msgs::Led>     ("/mobile_base/commands/led2", 1);
  sound_pub_ = nh_.advertise <kobuki_msgs::Sound>   ("/mobile_base/commands/sound", 1);
  table_marker_pub_  = nh_.advertise <visualization_msgs::MarkerArray> ("/table_marker", 1, true);

  digital_input_sub_ = nh_.subscribe("digital_input",   5, &WaiterNode::digitalInputCB, this);
  core_sensors_sub_  = nh_.subscribe("core_sensors",    5, &WaiterNode::coreSensorsCB, this);
  table_poses_sub_   = nh_.subscribe("table_pose_list", 1, &WaiterNode::tablePosesCB, this);

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
  //navigator_.enableSafety();

  return true;
}

void WaiterNode::spin()
{
  ros::Rate rate(50.0);

  // LED blinking stuff; TODO make a separate HRI class
  double blink_frequency = 2.0;
  double last_blink_time = 0.0;
  uint8_t last_blink_led  = 2;
  kobuki_msgs::Led green_led_msg, orange_led_msg, red_led_msg, off_led_msg;
  red_led_msg.value    = kobuki_msgs::Led::RED;
  off_led_msg.value    = kobuki_msgs::Led::BLACK;
  green_led_msg.value  = kobuki_msgs::Led::GREEN;
  orange_led_msg.value = kobuki_msgs::Led::ORANGE;

  while (ros::ok())
  {
    // Make LEDs blink to show current status
    double now = ros::Time::now().toSec();
    double delta_t = now - last_blink_time;
    if (delta_t > (1 / blink_frequency))
    {
      last_blink_time = now;
      last_blink_led = (last_blink_led % 2) + 1;

      switch (order_.status)
      {
        case cafe_msgs::Status::ERROR:
          led_1_pub_.publish(last_blink_led == 1 ?   off_led_msg :   red_led_msg);
          led_2_pub_.publish(last_blink_led == 1 ?   red_led_msg :   off_led_msg);
          break;
        case cafe_msgs::Status::WAITING_FOR_KITCHEN:
        case cafe_msgs::Status::WAITING_FOR_USER_CONFIRMATION:
          led_1_pub_.publish(last_blink_led == 1 ?   off_led_msg : green_led_msg);
          led_2_pub_.publish(last_blink_led == 1 ? green_led_msg :   off_led_msg);
          break;
        default:
          // Switch off LEDs by default
          led_1_pub_.publish(off_led_msg);
          led_2_pub_.publish(off_led_msg);
          break;
      }
    }

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

