/*
 * nav_watchdog.cpp
 *
 *  Created on: Apr 10, 2013
 *      Author: jorge
 */

#include <tf/tf.h>

#include "waiterbot/common.hpp"
//#include "waiterbot/navigator.hpp"
#include "waiterbot/nav_watchdog.hpp"


namespace waiterbot
{


NavWatchdog::NavWatchdog()
{
  localized_ = 0;         // initially lost
};

NavWatchdog::~NavWatchdog()
{
}

void NavWatchdog::arMarkerMsgCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  tf::Pose amcl_pose, armk_pose;
  tf::poseMsgToTF(last_amcl_pose_.pose, amcl_pose);
  tf::poseMsgToTF(msg->pose.pose,       armk_pose);
//
//  if (std::abs((last_amcl_init_.header.stamp - msg->header.stamp).toSec())  > 10.0)
//  {
//    geometry_msgs::PoseWithCovarianceStamped pose = *msg;
//    pose.header.stamp = ros::Time::now() + ros::Duration(0.1);
//    init_pose_pub_.publish(pose);
//    last_amcl_init_.header = msg->header;
//    last_amcl_init_.pose = msg->pose.pose;
//
//    ROS_WARN("Amcl (re)initialized by AR marker with pose: %.2f, %.2f, %.2f",
//              msg->pose.pose.position.x, msg->pose.pose.position.y, tf::getYaw(msg->pose.pose.orientation));
//return;
//  }


  if (! localized_)
  {
    ROS_DEBUG_THROTTLE(2.0, "AR marker localization received: %.2f, %.2f, %.2f",
                       msg->pose.pose.position.x, msg->pose.pose.position.y,
                       tf::getYaw(msg->pose.pose.orientation));
  }

  if ((! (localized_ & LOCALIZED_AMCL)) ||
      ((std::abs((last_amcl_pose_.header.stamp - msg->header.stamp).toSec())  < 1.0) &&
       (std::abs((last_amcl_init_.header.stamp - msg->header.stamp).toSec())  > 4.0) &&
       ((tk::distance2D(amcl_pose, armk_pose) > 1.0) || (tk::minAngle(amcl_pose, armk_pose) > 0.5))))
  {
    // If amcl has not received an initial pose from the user, or it's reporting a pose
    // far away from the one reported by the AR marker, initialize it with this message.
    // Note that we check timings to ensure we are not comparing with an old amcl pose

    // WARN/TODO: last_amcl_pose_ don't get updated with the robot stopped, so amcl will
    // not be reinitialized in that case.

    // Check if the covariance of the pose is low enough to use it to (re)initialize amcl
    const boost::array<double, 36u>& cov = msg->pose.covariance;
    if (max(cov[0], cov[1], cov[6], cov[7], cov[35]) > 0.05)
    {  // TODO: very arbitrary...
      ROS_WARN("Amcl not (re)initialized by AR marker because its cov. is not low enough");
      return;
    }

    geometry_msgs::PoseWithCovarianceStamped pose = *msg;
    pose.header.stamp = ros::Time::now() + ros::Duration(0.1);
    init_pose_pub_.publish(pose);
    last_amcl_init_.header = msg->header;
    last_amcl_init_.pose = msg->pose.pose;

    localized_ |= LOCALIZED_AMCL;

    ROS_WARN("Amcl (re)initialized by AR marker with pose: %.2f, %.2f, %.2f",
              msg->pose.pose.position.x, msg->pose.pose.position.y, tf::getYaw(msg->pose.pose.orientation));
    ROS_DEBUG("Cartesian distance = %.2f, Yaw difference = %.2f, TIME = %f",    // explain the reason
              tk::distance2D(amcl_pose, armk_pose), tk::minAngle(amcl_pose, armk_pose),
               (last_amcl_pose_.header.stamp - msg->header.stamp).toSec());

    // Reset costmaps, as they surely contains a lot of errors due to bad localization
    // TODO/WARN this will take effect BEFORE the relocalization, so the maps can get wrong again!!!
    //Navigator::getInstance().clearCostmaps();
  }
//  else
//  {
//    if (std::abs((last_amcl_pose_.header.stamp - msg->header.stamp).toSec())  >= 1.0)
//      ROS_DEBUG("POSE TOO OLD  %f", (last_amcl_pose_.header.stamp - msg->header.stamp).toSec());
//    if (std::abs((last_amcl_init_.header.stamp - msg->header.stamp).toSec())  <= 4.0)
//      ROS_DEBUG("INIT TOO RECENT %f", std::abs((last_amcl_init_.header.stamp - msg->header.stamp).toSec()));
//    if ((tk::distance2D(amcl_pose, armk_pose) <= 1.0) && (tk::minAngle(amcl_pose, armk_pose) <= 0.5))
//      ROS_DEBUG("POSE TOO CLOSE %f  %f", tk::distance2D(amcl_pose, armk_pose), tk::minAngle(amcl_pose, armk_pose));
//  }

  localized_ |= LOCALIZED_ARMK;
}

void NavWatchdog::amclPoseMsgCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  last_amcl_pose_.header = msg->header;
  last_amcl_pose_.pose = msg->pose.pose;

  // Amcl localization received; check how good it is
  const boost::array<double, 36u>& cov = msg->pose.covariance;
  double max_cov = max(cov[0], cov[1], cov[6], cov[7], cov[35]);

  if (max_cov > amcl_max_error_) {  // this is really a lot, so this case will only arise in extreme circumstances
    if (localized_ & LOCALIZED_AMCL) {
      ROS_WARN("Amcl is providing very inaccurate localization (max. cov. = %f)", max_cov);
      localized_ &= ~LOCALIZED_AMCL;

      // TODO recover:
      //  - amcl global pos
      //  - look for MK, wonder....
    }
  }
}

void NavWatchdog::newGoalMsgCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  // New goal designated; we just capture these events to provide log information
/* TODO  cannot be here anymore.... pero tampoco pega en cmd_vel_man...
  if (mode != MODE_AUTO) {
    ROS_WARN("New goal designated while not in automatic navigation mode");
  }
  else if (drop_risk == true) {
    ROS_WARN("Automatic navigation not allowed while the robot is on dropping risk");
  }
  else if (collision == true) {
    ROS_WARN("Automatic navigation not allowed while the robot is on collision");
  }
  else*/ if (localized_ == false) {
    ROS_WARN("Automatic navigation not allowed while the robot is not localized");
  }
  else {
    ROS_INFO("New goal designated: %.2f, %.2f, %.2f",
              msg->pose.position.x, msg->pose.position.y, tf::getYaw(msg->pose.orientation));
  }
}

void NavWatchdog::initPoseMsgCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  // Amcl (re)initialized by someone (cannot know who); consider it as localized
  localized_ |= LOCALIZED_AMCL;
}

//void NavWatchdog::feedbackMsgCB(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg)
//{
//
//}
//
//void NavWatchdog::glResultMsgCB(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg)
//{
//  // TODO provide feedback to the user
//}
//
//void NavWatchdog::glStatusMsgCB(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
//{
//
//}

bool NavWatchdog::init()
{
  ros::NodeHandle nh, pnh("~");

  // Parameters
  pnh.getParam("check_localized", check_localized_);
  if (check_localized_ == false)
  {
    // Ignore whether we are localized; useful in local navigation or mapping
    localized_ |= LOCALIZED_FAKE;
  }

  pnh.param("amcl_max_error", amcl_max_error_, 2.0);

  // Publishers and subscribers
  amcl_p_sub_ = nh.subscribe("amcl_pose", 1, &NavWatchdog::amclPoseMsgCB, this);
  init_p_sub_ = nh.subscribe("amcl_init", 1, &NavWatchdog::initPoseMsgCB, this);
  n_goal_sub_ = nh.subscribe("new_goal",  1, &NavWatchdog::newGoalMsgCB,  this);

  init_pose_pub_   = nh.advertise <geometry_msgs::PoseWithCovarianceStamped> ("reset_pose",  1);
  cancel_goal_pub_ = nh.advertise <actionlib_msgs::GoalID>                   ("cancel_goal", 1);

  ROS_INFO("Navigation watchdog successfully initialized");

  return true;
}

} /* namespace waiterbot */
