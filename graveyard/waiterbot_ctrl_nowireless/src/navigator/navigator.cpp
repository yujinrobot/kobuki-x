/*
  Navigator Class

  Highly inspired by navigator from waiterbot_controller written by Jorge Santos

  LICENSE : BSD - https://raw.github.com/yujinrobot/kobuki-x/license/LICENSE

  Author :  Jihoon Lee
  Date   : Dec 2013
 */

#include "waiterbot_ctrl_nowireless/navigator.hpp" 

namespace waiterbot {
  Navigator::Navigator(ros::NodeHandle& n) : 
    nh_(n),
    ac_move_base_(NavigatorDefaultParam::AC_MOVE_BASE,true)
  {
    if(init())
      initialized_ = true;
    else 
      initialized_ = false;
  }

  Navigator::~Navigator() {
  }

  bool Navigator::init() {
    // publisher
    pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>(NavigatorDefaultParam::PUB_CMD_VEL, 1);

    // subscriber
    sub_odometry_ = nh_.subscribe(NavigatorDefaultParam::SUB_ODOM, 5, &Navigator::odometryCB, this);

    // clear cost map service
    srv_clear_costmaps = nh_.serviceClient<std_srvs::Empty>(NavigatorDefaultParam::SRV_CLEAR_COSTMAP);

    // Wait for base_move 
    if(waitForServer(ac_move_base_, NavigatorDefaultParam::AC_MOVE_BASE, 15.0) == false)
    {
      ROS_ERROR("Waiter : Move-base action server not available...");
      return false;
    }

    return true;
  }

  void Navigator::odometryCB(const nav_msgs::Odometry::ConstPtr& msg)
  {
    odometry_ = *msg;
  }

  bool Navigator::isInit() 
  {
    return initialized_; 
  }

  template <typename T>
  bool Navigator::waitForServer(actionlib::SimpleActionClient<T>& action_client, const std::string ac_name, const double timeout = 2.0f)
  {
    // Wait for the required action servers to come up; the huge timeout is not a whim; move_base
    // action server can take up to 20 seconds to initialize in the official turtlebot laptop!
    ros::Time t0 = ros::Time::now();

    while ((action_client.waitForServer(ros::Duration(1.0)) == false) && (ros::ok() == true))
    {
      if ((ros::Time::now() - t0).toSec() > timeout/2.0)
        ROS_WARN_THROTTLE(3, "Navigator : Waiting for %s action server to come up...", ac_name.c_str());
    
      if ((ros::Time::now() - t0).toSec() > timeout)
      {
        ROS_ERROR("Navigator : Timeout while waiting for %s action server to come up", ac_name.c_str());
        return false;
      }
    } 
    return true;
  }
}
