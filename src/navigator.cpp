/*
 * navigator.cpp
 *
 *  Created on: Apr 11, 2013
 *      Author: jorge
 */

#include <actionlib_msgs/GoalID.h>
#include <move_base_msgs/MoveBaseAction.h>

#include "waiterbot/common.hpp"
#include "waiterbot/navigator.hpp"

namespace waiterbot
{

Navigator::Navigator() : NAME_("Navigator"), state_(IDLE),
    move_base_ac_("move_base", true),                // tell the action clients that we
    auto_dock_ac_("dock_drive_action", true)         // want to spin a thread by default  TODO sure???
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

  issue_goal_pub_  = nh.advertise <geometry_msgs::PoseStamped> ("issue_goal",  1);
  cancel_goal_pub_ = nh.advertise <actionlib_msgs::GoalID>     ("cancel_goal", 1);

  // Wait for the action server to come up
  if ((move_base_ac_.waitForServer(ros::Duration(5.0)) == false) ||
      (auto_dock_ac_.waitForServer(ros::Duration(5.0)) == false))
  {
    ROS_ERROR("Timeout while waiting for an action server to come up. Navigator is unavailable");
    return false;
  }

  return true;
}

void Navigator::baseSpottedMsgCB(const geometry_msgs::PoseStamped::ConstPtr& msg, uint32_t id)
{
  base_rel_pose_  = *msg;
  base_marker_id_ = id;
}

bool Navigator::dockInBase(const geometry_msgs::PoseStamped& base_abs_pose)
{
  if (base_abs_pose.header.frame_id != global_frame_)
  {
    ROS_ERROR("Docking base pose not in global frame (%s != %s)",
              base_abs_pose.header.frame_id.c_str(), global_frame_.c_str());
    return false;
  }

  // Project a pose in front of the docking base and heading to it
  ROS_INFO("Global navigation to docking base...");

  tf::StampedTransform marker_gb;   // docking base marker on global reference system
  tk::pose2tf(base_abs_pose, marker_gb);

  tf::Transform goal_gb(marker_gb); // global pose facing the base
  tf::Transform r(tf::createQuaternionFromRPY(-M_PI/2.0, 0.0, M_PI/2.0));
  tf::Transform t(tf::Quaternion::getIdentity(),
                  tf::Vector3(-base_marker_distance_/1.5, 0.0, -marker_gb.getOrigin().z()));

  marker_gb *= r;  // rotate to adopt mobile base Euler angles, facing the marker
  marker_gb *= t;  // translate to put on the ground at some distance of the marker

  // Note that goal is relative to map frame
  move_base_msgs::MoveBaseGoal mb_goal;
  tk::tf2pose(marker_gb, mb_goal.target_pose);

  ROS_DEBUG("Sending goal to robot: %.2f, %.2f, %.2f (relative to %s)",
            mb_goal.target_pose.pose.position.x, mb_goal.target_pose.pose.position.y,
            tf::getYaw(mb_goal.target_pose.pose.orientation), mb_goal.target_pose.header.frame_id.c_str());
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
      mb_goal.target_pose.pose.position.y = 0.0;
      mb_goal.target_pose.pose.position.z = base_beacon_distance_;
      mb_goal.target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);

      ROS_DEBUG("Sending goal to robot: %.2f, %.2f, %.2f (relative to %s)",
                mb_goal.target_pose.pose.position.x, mb_goal.target_pose.pose.position.y,
                tf::getYaw(mb_goal.target_pose.pose.orientation), mb_goal.target_pose.header.frame_id.c_str());
      move_base_ac_.sendGoal(mb_goal);

      state_ = MARKER_DOCKING;
    }
    else
    {
      ROS_DEBUG_THROTTLE_NAMED(2.0, NAME_, "%s", move_base_ac_.getState().getText().c_str());
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
    ROS_WARN_NAMED(NAME_, "Unable to spot docking base marker within the required distance (last spot was %f s ago at %f m)",
                   (ros::Time::now() - base_rel_pose_.header.stamp).toSec(), base_rel_pose_.pose.position.z);
    // TODO; why we cannot see the marker??? that's bad... probably autodocking will fail but we try anyway
  }

  // We should be in front of the docking base at base_beacon_distance_; switch on auto-docking
  ROS_INFO("Switching on beacon-based auto-docking...");
  kobuki_msgs::AutoDockingGoal ad_goal;
  auto_dock_ac_.sendGoal(ad_goal);

  while (auto_dock_ac_.waitForResult(ros::Duration(2.0)) == false)
  {
    ROS_DEBUG_THROTTLE_NAMED(2.0, NAME_, "%s", auto_dock_ac_.getState().getText().c_str());
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
