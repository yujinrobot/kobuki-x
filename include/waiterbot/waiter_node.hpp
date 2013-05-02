 /*
 * waiter_node.hpp
 *
 *  Created on: Apr 6, 2013
 *      Author: jorge
 */

#ifndef WAITER_NODE_HPP_
#define WAITER_NODE_HPP_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <kobuki_msgs/SensorState.h>

#include <cafe_msgs/Status.h>
#include <cafe_msgs/Order.h>
#include <cafe_msgs/DeliverOrderAction.h>
#include <semantic_region_handler/TablePoseList.h>

#include "waiterbot/ar_markers.hpp"
#include "waiterbot/nav_watchdog.hpp"
#include "waiterbot/navigator.hpp"

namespace waiterbot
{

class WaiterNode
{
public:

  WaiterNode(std::string name) :
    as_(nh_, "delivery_order", false),
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
  bool wakeUp();
  bool leaveNest();

  void odometryCB(const nav_msgs::Odometry::ConstPtr& msg);
  void coreSensorsCB(const kobuki_msgs::SensorState::ConstPtr& msg);
  void tablePosesCB(const semantic_region_handler::TablePoseList::ConstPtr& msg);

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
  cafe_msgs::DeliverOrderResult   result_;

  /*********************
  ** Publishers
  **********************/
  ros::Publisher led_1_pub_;
  ros::Publisher led_2_pub_;
  ros::Publisher sound_pub_;

  /*********************
  ** Subscribers
  **********************/
  ros::Subscriber core_sensors_sub_;
  ros::Subscriber table_poses_sub_;

  ARMarkers   ar_markers_;
  NavWatchdog nav_watchd_;
  Navigator   navigator_;

  geometry_msgs::PoseStamped             pickup_pose_;
  semantic_region_handler::TablePoseList table_poses_;
//  ar_track_alvar::AlvarMarker base_marker_;
//  uint16_t  dock_marker_;   /**< AR marker identifying this robot's docking station */
  kobuki_msgs::SensorState core_sensors_;
  cafe_msgs::Order  order_;
  cafe_msgs::Status status_;


  boost::thread order_process_thread_;

  bool initialized_;
  bool initialized_table_;

  bool processOrder(cafe_msgs::Order& order);
  bool getReadyToWork();
  bool waitForPoses();
  bool waitForButton();
  bool gotoTable(int table_id);
  void sendFeedback(int feedback_status);
  bool setFailure(std::string reason);

  bool cleanupAndSuccess();
  bool cleanupAndError();
};

} /* namespace waiterbot */

#endif /* WAITER_NODE_HPP_ */
