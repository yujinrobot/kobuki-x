/*
  Nav Watchdog Class

  Highly inspired by nav_watchdog from waiterbot_controller written by Jorge Santos Simon
  It watches localization and relocalize based on global ar markers

  LICENSE : BSD - https://raw.github.com/yujinrobot/kobuki-x/license/LICENSE

  Author :  Jorge Santos
  Date   : Apr 10,2013

  Modified : Jihoon Lee
  Date   : Def, 2013
 */

#include "waiterbot_ctrl_nowireless/nav_watchdog.hpp"

namespace waiterbot {
  NavWatchdog::NavWatchdog(ros::NodeHandle& n) : nh_(n) {}
  NavWatchdog::~NavWatchdog() {}

  bool NavWatchdog::init() 
  {
    ros::NodeHandle priv_n("~");
    priv_n.param("nav_watchdog/amcl_max_error", amcl_max_error_, NavWatchdogDefaultParam::AMCL_MAX_ERROR);


    pub_init_pose_     = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped> (NavWatchdogDefaultParam::PUB_INIT_POSE, 1);

    sub_robot_pose_ar_ = nh_.subscribe(NavWatchdogDefaultParam::SUB_ROBOT_POSE_AR, 1, &NavWatchdog::robotPoseARCB, this); 
    sub_init_pose_     = nh_.subscribe(NavWatchdogDefaultParam::SUB_INIT_POSE, 1, &NavWatchdog::initPoseMsgCB, this);
    sub_amcl_pose_     = nh_.subscribe(NavWatchdogDefaultParam::SUB_AMCL_POSE, 1, &NavWatchdog::amclPoseMsgCB, this);
  
    return true;
  }

  void NavWatchdog::robotPoseARCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {

    tf::Pose amcl_pose, armk_pose;
    tf::poseMsgToTF(last_amcl_pose_.pose, amcl_pose);
    tf::poseMsgToTF(msg->pose.pose,       armk_pose);
  
    if (! localized_)
    {
      ROS_DEBUG_THROTTLE(2.0, "AR marker localization received: %.2f, %.2f, %.2f", msg->pose.pose.position.x, msg->pose.pose.position.y, tf::getYaw(msg->pose.pose.orientation));
    }
  
    if ((! (localized_ & LOCALIZED_AMCL)) ||
        ((std::abs((last_amcl_pose_.header.stamp - msg->header.stamp).toSec())  < 1.0) &&
         (std::abs((last_amcl_init_.header.stamp - msg->header.stamp).toSec())  > 4.0) &&
         ((mtk::distance2D(amcl_pose, armk_pose) > 1.0) || (mtk::minAngle(amcl_pose, armk_pose) > 0.5))))
    {
      // If amcl has not received an initial pose from the user, or it's reporting a pose
      // far away from the one reported by the AR marker, initialize it with this message.
      // Note that we check timings to ensure we are not comparing with an old amcl pose
  
      // WARN/TODO: last_amcl_pose_ don't get updated with the robot stopped, so amcl will
      // not be reinitialized in that case.
  
      // Check if the covariance of the pose is low enough to use it to (re)initialize amcl
      const boost::array<double, 36u>& cov = msg->pose.covariance;
      if (MAX5(cov[0], cov[1], cov[6], cov[7], cov[35]) > 0.05)
      {  // TODO: very arbitrary...
        ROS_WARN("Amcl not (re)initialized by AR marker because its cov. is not low enough");
        return;
      }
  
      geometry_msgs::PoseWithCovarianceStamped pose = *msg;
      pose.header.stamp = ros::Time::now() + ros::Duration(0.1);
      pub_init_pose_.publish(pose);
      last_amcl_init_.header = msg->header;
      last_amcl_init_.pose = msg->pose.pose;
  
      localized_ |= LOCALIZED_AMCL;
  
      ROS_WARN("Amcl (re)initialized by AR marker with pose: %.2f, %.2f, %.2f",
                msg->pose.pose.position.x, msg->pose.pose.position.y, tf::getYaw(msg->pose.pose.orientation));
      ROS_DEBUG("Cartesian distance = %.2f, Yaw difference = %.2f, TIME = %f",    // explain the reason
                mtk::distance2D(amcl_pose, armk_pose), mtk::minAngle(amcl_pose, armk_pose),
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
  //    if ((mtk::distance2D(amcl_pose, armk_pose) <= 1.0) && (mtk::minAngle(amcl_pose, armk_pose) <= 0.5))
  //      ROS_DEBUG("POSE TOO CLOSE %f  %f", mtk::distance2D(amcl_pose, armk_pose), mtk::minAngle(amcl_pose, armk_pose));
  //  }
  
    localized_ |= LOCALIZED_ARMK;
  }

  void NavWatchdog::amclPoseMsgCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    last_amcl_pose_.header = msg->header;
    last_amcl_pose_.pose   = msg->pose.pose;

    // amcl localization received; check how good it is
    const boost::array<double, 36u>& cov = msg->pose.covariance;
    double max_cov = MAX5(cov[0], cov[1], cov[6], cov[7], cov[35]);

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

  void NavWatchdog::initPoseMsgCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    // Amcl (re)initialized by someone (cannot know who); consider it as localized
    localized_ |= LOCALIZED_AMCL;
  }
}
