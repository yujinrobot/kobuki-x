/*
 * navigator.hpp
 *
 *  Created on: Apr 11, 2013
 *      Author: jorge
 */

#ifndef NAVIGATOR_HPP_
#define NAVIGATOR_HPP_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <kobuki_msgs/AutoDockingAction.h>

namespace waiterbot
{

class Navigator
{
public:
  const std::string NAME_;  /**< Waiterbot node sub-module name */

  enum
  {
    IDLE,
    GLOBAL_DOCKING,
    MARKER_DOCKING,
    BEACON_DOCKING

  } state_;

  Navigator();
  virtual ~Navigator();

  bool init();

  bool dockInBase(const geometry_msgs::PoseStamped& base_abs_pose);

  void baseSpottedMsgCB(const geometry_msgs::PoseStamped::ConstPtr& msg, uint32_t id);

private:
  std::string global_frame_;
  std::string odom_frame_;
  std::string base_frame_;

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_ac_;
  actionlib::SimpleActionClient<kobuki_msgs::AutoDockingAction> auto_dock_ac_;

  double base_beacon_distance_;  /**< At which distance from the base we start relaying on ir beacons */
  double base_marker_distance_;  /**< At which distance from the base we start relaying on ar markers */

  uint32_t                   base_marker_id_;
  geometry_msgs::PoseStamped base_rel_pose_;  /**< Docking base ar marker pose relative to the robot */

  ros::Publisher issue_goal_pub_;
  ros::Publisher cancel_goal_pub_;
};

} /* namespace waiterbot */
#endif /* NAVIGATOR_HPP_ */
