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
  const double GO_TO_POSE_TIMEOUT;

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

  void enableSafety() { safety_on_pub_.publish(std_msgs::Empty()); };
  void disableSafety() { safety_off_pub_.publish(std_msgs::Empty()); };

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

  ros::Publisher goal_poses_pub_;
  ros::Publisher issue_goal_pub_;
  ros::Publisher cancel_goal_pub_;
  ros::Publisher safety_on_pub_;
  ros::Publisher safety_off_pub_;


  template <typename T>
  bool waitForServer(actionlib::SimpleActionClient<T> & action_client, double timeout = 2.0)
  {
    // Wait for the required action servers to come up; the huge timeout is not a whim; move_base
    // action server can take up to 20 seconds to initialize in the official turtlebot laptop!
    ros::Time t0 = ros::Time::now();

    while ((action_client.waitForServer(ros::Duration(1.0)) == false) && (ros::ok() == true))
    {
      if ((ros::Time::now() - t0).toSec() > timeout/2.0)
        ROS_WARN_THROTTLE(3, "Waiting for action server to come up...");

      if ((ros::Time::now() - t0).toSec() > timeout)
      {
        ROS_ERROR("Timeout while waiting for required action server to come up");
        return false;
      }
    }

    return true;
  }
};

} /* namespace waiterbot */
#endif /* NAVIGATOR_HPP_ */
