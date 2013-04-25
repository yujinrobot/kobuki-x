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
#include "waiterbot/nav_watchdog.hpp"
#include "waiterbot/navigator.hpp"

namespace waiterbot
{

class WaiterNode
{
public:

  WaiterNode(std::string name) :
    as_(nh_, "deliver_order", false),
    node_name_(name),
    SPOT_BASE_MARKER_TIMEOUT(10.0),
    SPOT_POSE_MARKER_TIMEOUT(15.0)
  {
  }

  ~WaiterNode(void)
  {
  }

  bool init();
  void spin();

  void odometryCB(const nav_msgs::Odometry::ConstPtr& msg);
  void coreSensorsCB(const kobuki_msgs::SensorState::ConstPtr& msg);

  void deliverOrderCB();
  void preemptOrderCB();

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
  ros::Subscriber odometry_sub_;

  ARMarkers   ar_markers_;
  NavWatchdog nav_watchd_;
  Navigator   navigator_;

  ar_track_alvar::AlvarMarker base_marker_;
  uint16_t  dock_marker_;   /**< AR marker identifying this robot's docking station */
  kobuki_msgs::SensorState core_sensors_;
  nav_msgs::Odometry   odometry_;
  cafe_msgs::Order  order_;
  cafe_msgs::Status status_;

  void wakeUp();

  /*********************
  ** Simple commands
  **********************/

  void slowForward()
  {
    moveAt( 0.1,  0.0,  0.1);
  }

  void slowBackward()
  {
    moveAt(-0.1,  0.0,  0.1);
  }

  void turnClockwise()
  {
    moveAt( 0.0, -0.5,  0.1);
  }

  void turnCounterClockwise()
  {
    moveAt( 0.0,  0.5,  0.1);
  }

  void stop()
  {
    moveAt( 0.0,  0.0,  0.0);
  }

  void moveAt(double v, double w, double t = 0.0)
  {
    geometry_msgs::Twist vel;
    vel.linear.x  = v;
    vel.angular.z = w;
    cmd_vel_pub_.publish(vel);
    ros::Duration(t).sleep();
  }

};

} /* namespace waiterbot */

#endif /* WAITER_NODE_HPP_ */
