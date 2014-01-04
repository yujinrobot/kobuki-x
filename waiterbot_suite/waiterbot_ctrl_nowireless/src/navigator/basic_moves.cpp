/*
  Navigator Class

  Highly inspired by navigator from waiterbot_controller written by Jorge Santos

  LICENSE : BSD - https://raw.github.com/yujinrobot/kobuki-x/license/LICENSE

  Author :  Jihoon Lee
  Date   : Dec 2013
 */
#include "waiterbot_ctrl_nowireless/navigator.hpp" 

namespace waiterbot {
  void Navigator::slowForward()
  {
    moveAt( 0.1,  0.0,  0.1);
  }

  void Navigator::slowBackward()
  {
    moveAt(-0.1,  0.0,  0.1);
  }

  void Navigator::turnClockwise()
  {
    moveAt( 0.0, -0.5,  0.1);
  }

  void Navigator::turnCounterClockwise()
  {
    moveAt( 0.0,  0.5,  0.1);
  }

  void Navigator::stop()
  {
    moveAt( 0.0,  0.0,  0.0);
  }

  void Navigator::moveAt(double v, double w, double t = 0.0)
  {
    geometry_msgs::Twist vel;
    vel.linear.x  = v;
    vel.angular.z = w;
    pub_cmd_vel_.publish(vel);
    ros::Duration(t).sleep();
  }

  void Navigator::forward(double distance)
  {
    geometry_msgs::Point pos0 = odometry_.pose.pose.position;
    while (mtk::distance2D(pos0, odometry_.pose.pose.position) < distance)
      slowForward();
  }

  void Navigator::backward(double distance)
  {
    geometry_msgs::Point pos0 = odometry_.pose.pose.position;
    while (mtk::distance2D(pos0, odometry_.pose.pose.position) < distance)
      slowBackward();
  }

  void Navigator::turn(double angle)
  {
    double yaw0 = tf::getYaw(odometry_.pose.pose.orientation);
    double yaw1 = mtk::wrapAngle(yaw0 + angle);

    ROS_DEBUG("%f  %f  %f", angle, yaw0, yaw1);
    while (std::abs(mtk::wrapAngle(yaw1 - tf::getYaw(odometry_.pose.pose.orientation))) > 0.05)
      moveAt(0.0, mtk::sign(angle)*0.5, 0.05);
  }

  // aaaagh...  TODO make a decent version checking sign change and turns over 2pi! I have no time now
  void Navigator::turn2(double angle)
  {
    double yaw0 = tf::getYaw(odometry_.pose.pose.orientation);
    double yaw1 = mtk::wrapAngle(yaw0 + angle);
    double sign = mtk::sign(yaw1 - yaw0);

    ROS_DEBUG("%f  %f  %f", angle, yaw0, yaw1);
    while (mtk::sign(mtk::wrapAngle(tf::getYaw(odometry_.pose.pose.orientation) - yaw1)) == sign)
    {
      moveAt(0.0, mtk::sign(angle)*0.5, 0.05);
    }
  }

  void Navigator::spinClockwise()
  {
    // WARN; note that this requires that callbacks keep running!
    double yaw0 = tf::getYaw(odometry_.pose.pose.orientation);

    for (int i = 0; i < 5 || tf::getYaw(odometry_.pose.pose.orientation) <= yaw0; ++i)
      turnClockwise();

    for (int i = 0; i < 5 || tf::getYaw(odometry_.pose.pose.orientation) >  yaw0; ++i)
      turnClockwise();
  }

  void Navigator::spinCounterClockwise()
  {
    // WARN; note that this requires that callbacks keep running!
    double yaw0 = tf::getYaw(odometry_.pose.pose.orientation);

    for (int i = 0; i < 5 || tf::getYaw(odometry_.pose.pose.orientation) >= yaw0; ++i)
      turnCounterClockwise();

    for (int i = 0; i < 5 || tf::getYaw(odometry_.pose.pose.orientation) <  yaw0; ++i)
      turnCounterClockwise();
  }

  bool Navigator::moveTo(const geometry_msgs::PoseStamped& goal_pose)
  {
    // TODO :check if goal_pose is in global frame 

    ROS_INFO("Global navigation to point %.2f %.2f %.2f", goal_pose.pose.position.x, goal_pose.pose.position.y,tf::getYaw(goal_pose.pose.orientation));
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = goal_pose;

    ac_move_base_.sendGoal(goal);

    while(ac_move_base_.waitForResult(ros::Duration(0.5)) == false && ros::ok()) {}

    if(ac_move_base_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Succeed to arrive!");
      return true;
    }
    else {
      ROS_WARN("Failed to arrive...");
      return false;
    }
    return false;
  }

  bool Navigator::cancelMoveTo()
  {
    ROS_WARN("Navigator : Canceling goal with %s state...", ac_move_base_.getState().toString().c_str());
    ac_move_base_.cancelGoal();

    /*
    while(ac_move_base_.waitForResult(ros::Duration(0.1)) == false && ros::ok())
    {
      ROS_INFO("Navigator : Waiting move base to be canceled...");
      ros::Duration(1.0).sleep();
    }

    ROS_INFO("Navigator : Move base canceled..");
    */
    return true;
  }

  bool Navigator::clearCostMaps() {
    ros::Time t0 = ros::Time::now();                                                           
    std_srvs::Empty srv;                                                                       
                                                                                                 
    if (srv_clear_costmaps.call(srv) == true)
    {                                                                                          
      ROS_INFO("Successfully cleared costmaps (%f seconds)", (ros::Time::now() - t0).toSec()); 
      return true;                                                                             
    }                                                                                          
    else                                                                                       
    {                                                                                          
      ROS_ERROR("Failed to clear costmaps (%f seconds)", (ros::Time::now() - t0).toSec());     
      return false;                                                                            
    }                                                                                          
  }
}


