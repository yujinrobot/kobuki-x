 /*
 * waiter_node.hpp
 *
 *  Created on: Apr 6, 2013
 *      Author: jorge
 */

#ifndef WAITER_NODE_HPP_
#define WAITER_NODE_HPP_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/server/simple_action_server.h>

#include <kobuki_msgs/SensorState.h>

#include <cafe_msgs/Status.h>
#include <cafe_msgs/DeliverOrderAction.h>

#include "waiterbot/ar_markers.hpp"

namespace waiterbot
{

class WaiterNode
{
protected:
  const double SPOT_BASE_MARKER_TIMEOUT;
  const double SPOT_POSE_MARKER_TIMEOUT;

  ros::NodeHandle nh_;
  std::string node_name_;

  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<cafe_msgs::DeliverOrderAction> as_;

  // create messages that are used to published feedback/result
  cafe_msgs::DeliverOrderFeedback feedback_;
  cafe_msgs::DeliverOrderResult result_;

  /*********************
  ** Publishers
  **********************/
  ros::Publisher cmd_vel_pub_;
  ros::Publisher led_1_pub_;
  ros::Publisher led_2_pub_;
  ros::Publisher sound_pub_;

  /*********************
  ** Subscribers
  **********************/
  ros::Subscriber sensors_sub_;
  ros::Subscriber ar_pose_sub_;
  ros::Subscriber odometry_sub_;


  ARMarkers ar_markers_;
  uint16_t  dock_marker_;   /**< AR marker identifying this robot's docking station */
  ar_track_alvar::AlvarMarkers global_markers_;  /**< AR markers described in the semantic map */
  kobuki_msgs::SensorState core_sensors_;
  nav_msgs::Odometry   odometry_;
  cafe_msgs::Order  order_;
  cafe_msgs::Status status_;

  void wakeUp();

public:

  WaiterNode(std::string name) :
    as_(nh_, name, boost::bind(&WaiterNode::deliveryCB, this, _1), false),
    node_name_(name),
    SPOT_BASE_MARKER_TIMEOUT(5.0),
    SPOT_POSE_MARKER_TIMEOUT(8.0)
  {
  }

  ~WaiterNode(void)
  {
  }

  bool init();

  void coreSensorsCB(const kobuki_msgs::SensorState::ConstPtr& msg);
  void odometryCB(const nav_msgs::Odometry::ConstPtr& msg);
  void deliveryCB(const cafe_msgs::DeliverOrderGoal::ConstPtr &goal);
};

} /* namespace waiterbot */

#endif /* WAITER_NODE_HPP_ */
