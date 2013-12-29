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

#ifndef _WAITER_NODE_NAV_WATCHDOG_HPP_
#define _WAITER_NODE_NAV_WATCHDOG_HPP_

#include <ros/ros.h>
#include <tf/tf.h>
#include <yocs_math_toolkit/geometry.hpp>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "default_params.hpp"

namespace waiterbot {

  #define LOCALIZED_ARMK   0x0001
  #define LOCALIZED_AMCL   0x0002
  #define LOCALIZED_FAKE   0x0004   // Ignore whether we are localized (local nav. or mapping

  #define sign(x) (x < 0.0? -1.0:+1.0)
  #define MAX(a,b)  ( std::max(a,b))
  #define MAX3(a,b,c)  ( MAX(MAX(a,b),c))
  #define MAX4(a,b,c,d) ( MAX(MAX3(a,b,c),d) )
  #define MAX5(a,b,c,d,e) ( MAX(MAX4(a,b,c,d),e) )

  class NavWatchdog
  {
    public:
      NavWatchdog(ros::NodeHandle& n);
      ~NavWatchdog();

      bool init();
    protected:
      void robotPoseARCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
      void amclPoseMsgCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
      void initPoseMsgCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    private:
      ros::NodeHandle nh_;
      ros::Publisher  pub_init_pose_;
      ros::Subscriber sub_robot_pose_ar_;
      ros::Subscriber sub_init_pose_;
      ros::Subscriber sub_amcl_pose_;

      geometry_msgs::PoseStamped last_amcl_pose_;
      geometry_msgs::PoseStamped last_amcl_init_;

      double amcl_max_error_;
      uint16_t localized_;
      bool check_localized_;
  };
}


#endif
