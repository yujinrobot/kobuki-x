/*
 * navigator.hpp
 *
 *  Created on: Apr 11, 2013
 *      Author: jorge
 */

#ifndef NAVIGATOR_HPP_
#define NAVIGATOR_HPP_

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>
#include <actionlib/client/simple_action_client.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <kobuki_msgs/AutoDockingAction.h>

#include <semantic_region_handler/TablePose.h>


namespace waiterbot
{

class Navigator
{
public:
  const double GO_TO_POSE_TIMEOUT;
  const double AUTO_DOCKING_TIMEOUT;
  const double WAIT_FOR_PICKUP_POINT;

  enum
  {
    IDLE,
    GLOBAL_DOCKING,
    MARKER_DOCKING,
    BEACON_DOCKING,
    GOING_TO_PICKUP,
    WAIT_FOR_PICKUP,
    GOING_TO_TABLE
//    WAIT_FOR_DELIVER


  } state_;

  Navigator();
  virtual ~Navigator();

  bool init();


  void enableMotors();
  void disableMotors();

  void enableSafety() { safety_on_pub_.publish(std_msgs::Empty()); };
  void disableSafety() { safety_off_pub_.publish(std_msgs::Empty()); };

  bool dockInBase(const geometry_msgs::PoseStamped& base_abs_pose);
  bool pickUpOrder(const geometry_msgs::PoseStamped& pick_up_pose);
  bool deliverOrder(const geometry_msgs::PoseStamped& table_pose, double table_radius);

  void odometryCB(const nav_msgs::Odometry::ConstPtr& msg);
  void baseSpottedMsgCB(const geometry_msgs::PoseStamped::ConstPtr& msg, uint32_t id);


  /*********************    TODO  crear otra clase para esta caquilla
  ** Basic move orders
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

  void forward(double distance)
  {
    geometry_msgs::Point pos0 = odometry_.pose.pose.position;
    while (tk::distance(pos0, odometry_.pose.pose.position) < distance)
      slowForward();
  }

  void backward(double distance)
  {
    geometry_msgs::Point pos0 = odometry_.pose.pose.position;
    while (tk::distance(pos0, odometry_.pose.pose.position) < distance)
      slowBackward();
  }

  void turn(double angle)
  {
    /*
    double angle360 = angle;

    while (angle360 >= + 2.0*M_PI)
    {
      spinCounterClockwise();
      angle360 -= 2.0*M_PI;
    }
    while (angle360 <= - 2.0*M_PI)
    {
      spinClockwise();
      angle360 += 2.0*M_PI;
    }
ignore by now turns bigger than 2pi
*/

    double yaw0 = tf::getYaw(odometry_.pose.pose.orientation);
    double yaw1 = tk::wrapAngle(yaw0 + angle);

    ROS_DEBUG("%f  %f  %f", angle, yaw0, yaw1);
    while (std::abs(tk::wrapAngle(yaw1 - tf::getYaw(odometry_.pose.pose.orientation))) > 0.05)
      moveAt(0.0, tk::sign(angle)*0.5, 0.05);
  }

  // aaaagh...  TODO make a decent version checking sign change and turns over 2pi! I have no time now
  void turn2(double angle)
  {
    double yaw0 = tf::getYaw(odometry_.pose.pose.orientation);
    double yaw1 = tk::wrapAngle(yaw0 + angle);
    double sign = tk::sign(yaw1 - yaw0);

    ROS_DEBUG("%f  %f  %f", angle, yaw0, yaw1);
    while (tk::sign(tk::wrapAngle(tf::getYaw(odometry_.pose.pose.orientation) - yaw1)) == sign){
//           (std::abs(tf::getYaw(odometry_.pose.pose.orientation) - yaw1) > 2.0*M_PI - 0.01)){//    ROS_DEBUG("%f > %f", tk::wrap_360(tf::getYaw(odometry_.pose.pose.orientation)), tk::wrap_360(yaw0 + angle360));
      moveAt(0.0, tk::sign(angle)*0.5, 0.05);
    }
  }

  void spinClockwise()
  {
    // WARN; note that this requires that callbacks keep running!
    double yaw0 = tf::getYaw(odometry_.pose.pose.orientation);

    for (int i = 0; i < 5 || tf::getYaw(odometry_.pose.pose.orientation) <= yaw0; ++i)
      turnClockwise();

    for (int i = 0; i < 5 || tf::getYaw(odometry_.pose.pose.orientation) >  yaw0; ++i)
      turnClockwise();
  }

  void spinCounterClockwise()
  {
    // WARN; note that this requires that callbacks keep running!
    double yaw0 = tf::getYaw(odometry_.pose.pose.orientation);

    for (int i = 0; i < 5 || tf::getYaw(odometry_.pose.pose.orientation) >= yaw0; ++i)
      turnCounterClockwise();

    for (int i = 0; i < 5 || tf::getYaw(odometry_.pose.pose.orientation) <  yaw0; ++i)
      turnCounterClockwise();
  }

                                                                                                   bool moveBaseReset();
private:
  bool       play_sounds_;
  double     robot_radius_;
  std::string global_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string resources_path_;

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_ac_;
  actionlib::SimpleActionClient<kobuki_msgs::AutoDockingAction> auto_dock_ac_;

  double relay_on_beacon_distance_;    /**< At which distance from the base we start relaying on ir beacons */
  double relay_on_marker_distance_;    /**< At which distance from the base we start relaying on ar markers */
  double close_to_pickup_distance_;    /**< At which distance from pickup point switch off recovery behavior */
  double close_to_delivery_distance_;  /**< At which distance from delivery point switch off recovery behavior */
  double tables_serving_distance_;     /**< At which distance from the table we try to serve our orders */

  uint32_t                   base_marker_id_;
  geometry_msgs::PoseStamped base_rel_pose_;  /**< Docking base ar marker pose relative to the robot */
  tf::TransformListener      tf_listener_;
  tf::TransformBroadcaster   tf_brcaster_;
  nav_msgs::Odometry         odometry_;
  bool                      recovery_behavior_;    /**< Whether recovery_behavior is enabled or not */

  ros::Subscriber odometry_sub_;

  ros::Publisher  cmd_vel_pub_;
  ros::Publisher  motor_pwr_pub_;
  ros::Publisher  goal_poses_pub_;
  ros::Publisher  issue_goal_pub_;
  ros::Publisher  cancel_goal_pub_;
  ros::Publisher  safety_on_pub_;
  ros::Publisher  safety_off_pub_;

  bool cleanupAndSuccess();
  bool cleanupAndError();
  bool enableRecovery();
  bool disableRecovery();
  bool shoftRecovery();
  bool hardRecovery();

  tf::StampedTransform getRobotTf();


  /********************************************************
  ** Template methods valid for all action clients
  *********************************************************/

  template <typename T>
  bool cancelAllGoals(actionlib::SimpleActionClient<T> & action_client, double timeout = 2.0)
  {
    actionlib::SimpleClientGoalState goal_state = action_client.getState();
    if ((goal_state != actionlib::SimpleClientGoalState::ACTIVE) &&
        (goal_state != actionlib::SimpleClientGoalState::PENDING) &&
        (goal_state != actionlib::SimpleClientGoalState::RECALLED) &&
        (goal_state != actionlib::SimpleClientGoalState::PREEMPTED))
    {
      // We cannot cancel a REJECTED, ABORTED, SUCCEEDED or LOST goal
      ROS_WARN("Cannot cancel %s goal, as it has %s state!", acName(action_client), goal_state.toString().c_str());
      return true;
    }

    ROS_INFO("Canceling %s goal with %s state...", acName(action_client), goal_state.toString().c_str());
    action_client.cancelAllGoals();
    if (action_client.waitForResult(ros::Duration(timeout)) == false)
    {
      ROS_WARN("Cancel %s goal didn't finish after %.2f seconds: %s",
               acName(action_client), timeout, goal_state.toString().c_str());
      return false;
    }

    ROS_WARN("Cancel %s goal succeed. New state is %s", acName(action_client), goal_state.toString().c_str());
    return true;
  }

  template <typename T>
  bool waitForServer(actionlib::SimpleActionClient<T> & action_client, double timeout = 2.0)
  {
    // Wait for the required action servers to come up; the huge timeout is not a whim; move_base
    // action server can take up to 20 seconds to initialize in the official turtlebot laptop!
    ros::Time t0 = ros::Time::now();

    while ((action_client.waitForServer(ros::Duration(1.0)) == false) && (ros::ok() == true))
    {
      if ((ros::Time::now() - t0).toSec() > timeout/2.0)
        ROS_WARN_THROTTLE(3, "Waiting for %s action server to come up...", acName(action_client));

      if ((ros::Time::now() - t0).toSec() > timeout)
      {
        ROS_ERROR("Timeout while waiting for %s action server to come up", acName(action_client));
        return false;
      }
    }

    return true;
  }

  template <typename T>
  const char* acName(actionlib::SimpleActionClient<T> & action_client)
  {
    return (typeid(T) == typeid(move_base_msgs::MoveBaseAction)) ? "move base" :
            (typeid(T) == typeid(kobuki_msgs::AutoDockingAction)) ? "auto-dock" : "ERROR";
  }
};

} /* namespace waiterbot */
#endif /* NAVIGATOR_HPP_ */
