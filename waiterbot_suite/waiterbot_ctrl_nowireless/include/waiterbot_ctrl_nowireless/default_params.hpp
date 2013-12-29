/*
  Waiter Default Params

  LICENSE : BSD - https://raw.github.com/yujinrobot/kobuki-x/license/LICENSE

  Author :  Jihoon Lee
  Date   : Dec 2013
 */

#ifndef _WAITER_CTRL_DEFAULT_PARAMS_
#define _WAITER_CTRL_DEFAULT_PARAMS_

#include <ros/ros.h>

namespace waiterbot {

  namespace WaiterIsolatedDefaultParam {
    const std::string LOC_VM                    = "loc_vm";
    const std::string LOC_CUSTOMER              = "loc_customer";
    const std::string BASE_FRAME                = "base_footprint";
    const std::string ODOM_FRAME                = "odom";
    const std::string GLOBAL_FRAME              = "map";  

    const std::string PUB_DRINK_ORDER_FEEDBACK  = "drink_order_feedback";

    const std::string SUB_DRINK_ORDER           = "drink_order";
    const std::string SUB_WAYPOINTS             = "waypoints";
    const std::string SUB_DIGITAL_INPUT         = "digital_input";
    const std::string AC_AUTODOCK               = "dock_drive_action";
  }

  namespace NavWatchdogDefaultParam {
    const std::string PUB_INIT_POSE     = "reset_pose";
    const std::string SUB_ROBOT_POSE_AR = "robot_pose_ar";
    const std::string SUB_INIT_POSE     = "amcl_init";
    const std::string SUB_AMCL_POSE     = "amcl_pose";
    
    const double AMCL_MAX_ERROR         = 2.0;
  }

  namespace NavigatorDefaultParam {
    const std::string AC_MOVE_BASE = "move_base";
    const std::string PUB_CMD_VEL  = "velocity";
    const std::string SUB_ODOM     = "odometry";
    const std::string SRV_CLEAR_COSTMAP = "move_base/clear_costmaps";
  }
}

#endif
