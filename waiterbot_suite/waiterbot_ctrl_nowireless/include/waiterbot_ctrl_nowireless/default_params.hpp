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

    const std::string PUB_DRINK_ORDER_FEEDBACK  = "status";
    const std::string SUB_DRINK_ORDER           = "goto";

    const std::string SUB_WAYPOINTS             = "waypoints";
    const std::string SUB_DIGITAL_INPUT         = "digital_input";
    const std::string AC_AUTODOCK               = "dock_drive_action";

    const std::string SUB_ORDER_CANCELLED       = "order_cancelled";
    const std::string SUB_TRAY_EMPTY            = "tray_empty";
  }

  namespace NavWatchdogDefaultParam {
    const std::string PUB_INIT_POSE             = "nav_watchdog/reset_pose";
    const std::string SUB_ROBOT_POSE_AR         = "nav_watchdog/robot_pose_ar";
    const std::string SUB_INIT_POSE             = "nav_watchdog/amcl_init";
    const std::string SUB_AMCL_POSE             = "nav_watchdog/amcl_pose";
    
    const double AMCL_MAX_ERROR                 = 2.0;
  }

  namespace NavigatorDefaultParam {
    const std::string AC_MOVE_BASE              = "move_base";
    const std::string PUB_CMD_VEL               = "velocity";
    const std::string SUB_ODOM                  = "odometry";
    const std::string SRV_CLEAR_COSTMAP         = "move_base/clear_costmaps";
  }
}

#endif
