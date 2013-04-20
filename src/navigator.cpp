/*
 * navigator.cpp
 *
 *  Created on: Apr 11, 2013
 *      Author: jorge
 */

#include <std_msgs/Empty.h>
#include <actionlib_msgs/GoalID.h>
#include <move_base_msgs/MoveBaseAction.h>

#include "waiterbot/common.hpp"
#include "waiterbot/navigator.hpp"

namespace waiterbot
{

Navigator::Navigator() : NAME_("Navigator"), state_(IDLE),
    move_base_ac_("move_base", true),                // tell the action clients that we
    auto_dock_ac_("dock_drive_action", true),        // want to spin a thread by default  TODO sure???
    base_marker_id_(std::numeric_limits<uint32_t>::max())
{
}

Navigator::~Navigator()
{
}

bool Navigator::init()
{
  ros::NodeHandle nh, pnh("~");

  // Parameters
  pnh.param("global_frame", global_frame_, std::string("map"));
  pnh.param("odom_frame",   odom_frame_,   std::string("odom"));
  pnh.param("base_frame",   base_frame_,   std::string("base_footprint"));

  pnh.param("base_beacon_distance", base_beacon_distance_, 0.5);
  pnh.param("base_marker_distance", base_marker_distance_, 0.5);

  goal_poses_pub_  = nh.advertise <geometry_msgs::PoseStamped> ("goal_pose", 1, true);
  issue_goal_pub_  = nh.advertise <geometry_msgs::PoseStamped> ("issue_goal", 1);
  cancel_goal_pub_ = nh.advertise <actionlib_msgs::GoalID>     ("cancel_goal", 1);

  safety_on_pub_   = nh.advertise <std_msgs::Empty> ("navigation_safety_controller/disable", 1);
  safety_off_pub_  = nh.advertise <std_msgs::Empty> ("navigation_safety_controller/enable", 1);

  // Disable navigation safety controller until we start moving
  // NOTE: it must be disable after completing any mission

  ROS_DEBUG("1");
  std_msgs::Empty kk;
  safety_off_pub_.publish(kk);
  ROS_DEBUG("2");

  safety_off_pub_.publish(std_msgs::Empty());
  ROS_DEBUG("3");


  return true;
}

void Navigator::baseSpottedMsgCB(const geometry_msgs::PoseStamped::ConstPtr& msg, uint32_t id)
{
  base_rel_pose_  = *msg;
  base_marker_id_ = id;
}

bool Navigator::dockInBase(const geometry_msgs::PoseStamped& base_abs_pose)
{
  // Enable safety controller on normal navigation
  safety_on_pub_.publish(std_msgs::Empty());

  if (base_abs_pose.header.frame_id != global_frame_)
  {
    ROS_ERROR("Docking base pose not in global frame (%s != %s)",
              base_abs_pose.header.frame_id.c_str(), global_frame_.c_str());
    return false;
  }

  // Project a pose relative to map frame in front of the docking base and heading to it
  ROS_INFO("Global navigation to docking base...");

  // Compensate the vertical alignment of markers and put at ground level to adopt navistack goals format
  tf::Transform marker_gb(tf::createQuaternionFromYaw(tf::getYaw(base_abs_pose.pose.orientation) - M_PI/2.0),
                          tf::Vector3(base_abs_pose.pose.position.x, base_abs_pose.pose.position.y, 0.0));

  // Half turn and translate to put goal at some distance in front of the marker
  tf::Transform in_front(tf::createQuaternionFromYaw(M_PI),
                         tf::Vector3(base_marker_distance_/1.5, 0.0, 0.0));

  move_base_msgs::MoveBaseGoal mb_goal;
  tk::tf2pose(marker_gb*in_front, mb_goal.target_pose.pose);
  mb_goal.target_pose.header.stamp = ros::Time::now();
  mb_goal.target_pose.header.frame_id = base_abs_pose.header.frame_id;

  // Wait for move base action servers to come up; the huge timeout is not a whim; move_base
  // action server can take up to 20 seconds to initialize in the official turtlebot laptop,
  // so is this request is issued short after startup...
  // WARN: move base server is not started unless global costmap has a positive update frequency
  if (waitForServer(move_base_ac_, 10.0) == false)
  {
    ROS_ERROR("Move base action server not available; we cannot navigate!");
    return false;
  }

  ROS_DEBUG("Sending goal to robot: %.2f, %.2f, %.2f (relative to %s)",
            mb_goal.target_pose.pose.position.x, mb_goal.target_pose.pose.position.y,
            tf::getYaw(mb_goal.target_pose.pose.orientation), mb_goal.target_pose.header.frame_id.c_str());
  goal_poses_pub_.publish(mb_goal.target_pose);
  move_base_ac_.sendGoal(mb_goal);
  state_ = GLOBAL_DOCKING;

  while (move_base_ac_.waitForResult(ros::Duration(0.5)) == false)
  {
    if ((state_ == GLOBAL_DOCKING) &&
        ((ros::Time::now() - base_rel_pose_.header.stamp).toSec() < 0.5) &&
        (base_rel_pose_.pose.position.z <= base_marker_distance_))
    {
      ROS_INFO("Docking base spotted at %.2f m; switching to marker-based local navigation...",
               base_rel_pose_.pose.position.z);

      // Docking base marker spotted at base_marker_distance; switch to relative goal
      move_base_ac_.cancelGoal();

      // Send a goal at base_beacon_distance_ in front the marker
      char marker_frame[32];
      sprintf(marker_frame, "ar_marker_%d", base_marker_id_);

      mb_goal.target_pose.header.frame_id = marker_frame;
      mb_goal.target_pose.header.stamp = ros::Time::now();

      mb_goal.target_pose.pose.position.x = 0.0;
      mb_goal.target_pose.pose.position.y = 0.0; // - mb_goal.target_pose.pose.position.y;
      mb_goal.target_pose.pose.position.z = base_beacon_distance_;
      mb_goal.target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);
      mb_goal.target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-M_PI/2.0, M_PI/2.0, 0.0);

      ROS_DEBUG("Sending goal to robot: %.2f, %.2f, %.2f (relative to %s)",
                mb_goal.target_pose.pose.position.x, mb_goal.target_pose.pose.position.y,
                tf::getYaw(mb_goal.target_pose.pose.orientation), mb_goal.target_pose.header.frame_id.c_str());
      goal_poses_pub_.publish(mb_goal.target_pose);
      move_base_ac_.sendGoal(mb_goal);

      state_ = MARKER_DOCKING;
    }
    else
    {
      ROS_DEBUG_THROTTLE(4.0, "Move base action state: %s; %s", move_base_ac_.getState().toString().c_str(), move_base_ac_.getState().getText().c_str());
    }
  }

  if (move_base_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Switching to beacon-based auto-docking...");
  }
  else
  {
    ROS_WARN("Go to docking base failed: %s", move_base_ac_.getState().getText().c_str());
    return false;
  }

  if (state_ == GLOBAL_DOCKING)
  {
    ROS_WARN("Unable to spot docking base marker within the required distance");
    if (base_marker_id_ < AR_MARKERS_COUNT)
    {
      ROS_WARN("Last spot was %f seconds ago at %f meters",
               (ros::Time::now() - base_rel_pose_.header.stamp).toSec(), base_rel_pose_.pose.position.z);
    }
    // TODO; why we cannot see the marker??? that's bad... probably auto-docking will fail but we try anyway
  }

  // We should be in front of the docking base at base_beacon_distance_; switch on auto-docking

  // Wait for auto-docking action servers to come up (it should be already by now)
  if (waitForServer(auto_dock_ac_, 2.0) == false)
  {
    ROS_ERROR("Auto-docking action server not available; we cannot dock!");
    return false;
  }

  ROS_INFO("Switching on beacon-based auto-docking...");
  kobuki_msgs::AutoDockingGoal ad_goal;
  auto_dock_ac_.sendGoal(ad_goal);

  // Disable navigation safety controller on auto-docking to avoid bouncing against the base
  safety_off_pub_.publish(std_msgs::Empty());

  while (auto_dock_ac_.waitForResult(ros::Duration(2.0)) == false)
  {
    ROS_DEBUG_THROTTLE(4.0, "Auto-dock action state: %s", auto_dock_ac_.getState().toString().c_str());
  }

  if (auto_dock_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Successfully docked...  zzz...   zzz...");
    return true;
  }
  else
  {
    ROS_WARN("Go to docking base failed: %s", auto_dock_ac_.getState().getText().c_str());
    return false;
  }


//  while not client.wait_for_server(rospy.Duration(5.0)):
//    if rospy.is_shutdown(): return
//    print 'Action server is not connected yet. still waiting...'
//
//  goal = AutoDockingGoal();
//  client.send_goal(goal, doneCb, activeCb, feedbackCb)
//  print 'Goal: Sent.'
//  rospy.on_shutdown(client.cancel_goal)
//  client.wait_for_result()
//
//  #print '    - status:', client.get_goal_status_text()
//  return client.get_result()

}

//void Navigator::dockInBase(geometry_msgs::PoseStamped base_abs_pose)
//{
//  move_base_msgs::MoveBaseGoal goal;
//
//  //we'll send a goal to the robot to move 1 meter forward
//  goal.target_pose.header.frame_id = "base_link";
//  goal.target_pose.header.stamp = ros::Time::now();
//
//  goal.target_pose.pose.position.x = 1.0;
//  goal.target_pose.pose.orientation.w = 1.0;
//
//  ROS_INFO("Sending goal");
//  move_base_ac_.sendGoal(goal);
//
//  move_base_ac_.waitForResult();
//
//  if(move_base_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//    ROS_INFO("Hooray, the base moved 1 meter forward");
//  else
//    ROS_INFO("The base failed to move forward 1 meter for some reason");
//
//}


} /* namespace waiterbot */
