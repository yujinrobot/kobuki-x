/*
  Navigator Class

  Highly inspired by navigator from waiterbot_controller written by Jorge Santos

  LICENSE : BSD - https://raw.github.com/yujinrobot/kobuki-x/license/LICENSE

  Author :  Jihoon Lee
  Date   : Dec 2013
 */

#ifndef _WAITER_NODE_NAVIGATOR_HPP_
#define _WAITER_NODE_NAVIGATOR_HPP_
#include <ros/ros.h>
#include <tf/tf.h>

#include <yocs_math_toolkit/common.hpp>
#include <yocs_math_toolkit/geometry.hpp>

#include <actionlib/client/simple_action_client.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <std_srvs/Empty.h>

#include "default_params.hpp"

namespace waiterbot {
  class Navigator {
    public:
      Navigator(ros::NodeHandle& n);
      ~Navigator();
      bool init();
      bool isInit();

      bool clearCostMaps();

      // basic movements 
      bool moveTo(const geometry_msgs::PoseStamped& pose);
      bool cancelMoveTo();

      void moveAt(double v, double w, double t);

      void slowForward();
      void slowBackward();
      void turnClockwise();
      void turnCounterClockwise();
      void stop();

      void forward(double distance);
      void backward(double distance);
      void turn(double angle);

      void turn2(double angle);
      void spinClockwise();
      void spinCounterClockwise();
    protected:
      void odometryCB(const nav_msgs::Odometry::ConstPtr& msg);

      template <typename T>
      bool waitForServer(actionlib::SimpleActionClient<T>& action_client, const std::string ac_name, const double timeout);
    private:
      actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_move_base_;

      ros::NodeHandle nh_;
      ros::Publisher pub_cmd_vel_;
      ros::Subscriber sub_odometry_;
      ros::ServiceClient srv_clear_costmaps;

      nav_msgs::Odometry odometry_;

      bool initialized_;
  };
}


#endif  // navigator
