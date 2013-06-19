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
#include <kobuki_msgs/DigitalInputEvent.h>

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
    SPOT_POSE_MARKER_TIMEOUT(15.0),
    blink_frequency_(2.0),
    last_blink_time_(0.0),
    last_blink_led_(1)
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
  void digitalInputCB(const kobuki_msgs::DigitalInputEvent::ConstPtr& msg);
  void coreSensorsCB(const kobuki_msgs::SensorState::ConstPtr& msg);
  void tablePosesCB(const semantic_region_handler::TablePoseList::ConstPtr& msg);

  void deliverOrderCB();
  void preemptOrderCB();

protected:
  const double SPOT_BASE_MARKER_TIMEOUT;
  const double SPOT_POSE_MARKER_TIMEOUT;

  ros::NodeHandle nh_;
  std::string node_name_;

  // NodeHandle instance must be created before this line. Otherwise strange error may occur
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
  ros::Publisher table_marker_pub_;

  /*********************
  ** Subscribers
  **********************/
  ros::Subscriber digital_input_sub_;
  ros::Subscriber core_sensors_sub_;
  ros::Subscriber table_poses_sub_;

  ARMarkers   ar_markers_;
  NavWatchdog nav_watchd_;
  Navigator   navigator_;

  geometry_msgs::PoseStamped             pickup_pose_;
  semantic_region_handler::TablePoseList table_poses_;
  kobuki_msgs::SensorState core_sensors_;
  cafe_msgs::Order order_;
  std::string global_frame_;

  // LED blinking attributes; TODO make a separate HRI class
  double blink_frequency_;
  double last_blink_time_;
  uint8_t last_blink_led_;

  boost::thread order_process_thread_;

  bool debug_mode_;
  bool initialized_;
  bool initialized_table_;

  bool wait_for_button_;

  bool processOrder(cafe_msgs::Order& order);
  bool getReadyToWork();
  bool waitForPoses();
  bool waitForButton();
  bool gotoTable(int table_id);
  void sendFeedback(int feedback_status);
  bool setSucceeded(std::string message);
  bool setFailure(std::string message);

  bool cleanupAndSuccess();
  bool cleanupAndError();

  const std::string toStr(int16_t status)
  {
    return std::string(toCStr(status));
  }

  const char* toCStr(int16_t status)
  {
    switch (status)
    {
      case cafe_msgs::Status::IDLE                          : return "idle";
      case cafe_msgs::Status::GO_TO_KITCHEN                 : return "going to kitchen";
      case cafe_msgs::Status::ARRIVE_KITCHEN                : return "arrived to kitchen";
      case cafe_msgs::Status::WAITING_FOR_KITCHEN           : return "waiting for kitchen";
      case cafe_msgs::Status::IN_DELIVERY                   : return "going to table";
      case cafe_msgs::Status::ARRIVE_TABLE                  : return "arrived to table";
      case cafe_msgs::Status::WAITING_FOR_USER_CONFIRMATION : return "waiting for customer";
      case cafe_msgs::Status::COMPLETE_DELIVERY             : return "delivery completed";
      case cafe_msgs::Status::RETURNING_TO_DOCK             : return "going to base";
      case cafe_msgs::Status::END_DELIVERY_ORDER            : return "order completed";
      case cafe_msgs::Status::ERROR                         : return "error";
      default                                               : return "UNRECOGNIZED STATUS";
    }
  }

  //< DEBUG
  void fakeOrderForEasyDebugging(int order_id, int table_id);
  //> DEBUG
};

} /* namespace waiterbot */

#endif /* WAITER_NODE_HPP_ */
