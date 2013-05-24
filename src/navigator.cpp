/*
 * navigator.cpp
 *
 *  Created on: Apr 11, 2013
 *      Author: jorge
 */

#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>
#include <actionlib_msgs/GoalID.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <dynamic_reconfigure/Reconfigure.h>

#include <kobuki_msgs/MotorPower.h>

#include "waiterbot/common.hpp"
#include "waiterbot/ar_markers.hpp"

#include "waiterbot/navigator.hpp"

namespace waiterbot
{

Navigator::Navigator() :
    state_(IDLE),
    move_base_ac_("move_base", true),                // tell the action clients that we
    auto_dock_ac_("dock_drive_action", true),        // want to spin a thread by default  TODO sure???
    base_marker_id_(std::numeric_limits<uint32_t>::max()),
    recovery_behavior_(true)  // enabled by default; check base_local_planner_params.yaml if not
{
}

Navigator::~Navigator()
{
}

bool Navigator::init()
{
  ros::NodeHandle nh, pnh("~");

  // Parameters
  pnh.param("global_frame",   global_frame_,   std::string("map"));
  pnh.param("odom_frame",     odom_frame_,     std::string("odom"));
  pnh.param("base_frame",     base_frame_,     std::string("base_footprint"));
  pnh.param("play_sounds",    play_sounds_,    false);
  pnh.param("resources_path", resources_path_, std::string("/"));
  if (resources_path_[resources_path_.length() - 1] != '/')
    resources_path_ += "/";

  pnh.param("relay_on_beacon_distance", relay_on_beacon_distance_, 0.4);
  pnh.param("relay_on_marker_distance", relay_on_marker_distance_, 1.0);
  pnh.param("tables_serving_distance",  tables_serving_distance_,  0.4);
  pnh.param("go_to_pose_timeout",       go_to_pose_timeout_,     300.0);
  pnh.param("auto_docking_timeout",     auto_docking_timeout_,    90.0);
  pnh.param("wait_for_pickup_point",    wait_for_pickup_point_,    8.8);

//TODO  pnh.param("close_to_pickup_distance", close_to_pickup_distance_, 2.0);

  nh.getParam("move_base/local_costmap/robot_radius", robot_radius_);
  nh.getParam("move_base/local_costmap/width", close_to_pickup_distance_);
  nh.getParam("move_base/planner_frequency", default_planner_frequency_);

  // We must disable recovery behavior well before entering the local costmap
  close_to_pickup_distance_ = close_to_pickup_distance_/2.0 + 0.8;
  close_to_delivery_distance_ = close_to_pickup_distance_;

  odometry_sub_    = nh.subscribe("odometry", 5, &Navigator::odometryCB, this);

  cmd_vel_pub_     = nh.advertise <geometry_msgs::Twist>    ("mobile_base/commands/velocity_", 1);
  motor_pwr_pub_   = nh.advertise <kobuki_msgs::MotorPower> ("mobile_base/commands/motor_power", 1);


  goal_poses_pub_  = nh.advertise <geometry_msgs::PoseStamped> ("goal_pose", 1, true);
  issue_goal_pub_  = nh.advertise <geometry_msgs::PoseStamped> ("issue_goal", 1);
  cancel_goal_pub_ = nh.advertise <actionlib_msgs::GoalID>     ("cancel_goal", 1);

  safety_on_pub_   = nh.advertise <std_msgs::Empty> ("navigation_safety_controller/enable", 1, true);
  safety_off_pub_  = nh.advertise <std_msgs::Empty> ("navigation_safety_controller/disable", 1, true);

  // Disable navigation safety controller until we start moving
  // NOTE: it must be enabled after completing the wake-up. Note also that we use latched
  // topics, as this first message can arrive before the controller gets up and running
  disableSafety();

  return true;
}

void Navigator::odometryCB(const nav_msgs::Odometry::ConstPtr& msg)
{
  odometry_ = *msg;
}

void Navigator::baseSpottedMsgCB(const geometry_msgs::PoseStamped::ConstPtr& msg, uint32_t id)
{
  base_rel_pose_  = *msg;
  base_marker_id_ = id;
}

bool Navigator::dockInBase()
{
  // This is the fallback solution in case of not having recognized base's AR marker
  ROS_WARN("Trying to go to docking base without recognizing its AR marker; not an easy business...");

  // Get latest map -> odom tf and displace slightly away from the base
  tf::StampedTransform odom_gb = getOdomTf();
  tf::Transform pull_back(tf::Quaternion::getIdentity(),
                          tf::Vector3(- relay_on_beacon_distance_, 0.0, 0.0));
  odom_gb *= pull_back;

  move_base_msgs::MoveBaseGoal mb_goal;
  tk::tf2pose(odom_gb, mb_goal.target_pose.pose);
  mb_goal.target_pose.header.stamp = ros::Time::now();
  mb_goal.target_pose.header.frame_id = global_frame_;

  return dockInBase_(mb_goal);
}

bool Navigator::dockInBase(const geometry_msgs::PoseStamped& base_marker_pose)
{
  if (base_marker_pose.header.frame_id != global_frame_)
  {
    ROS_ERROR("Docking base pose not in global frame (%s != %s)",
              base_marker_pose.header.frame_id.c_str(), global_frame_.c_str());
    return false;
  }

  // Get latest robot global pose
  tf::StampedTransform robot_gb = getRobotTf();

  // Project a pose relative to map frame in front of the docking base and heading to it
  ROS_INFO("Global navigation to docking base...");

  // Compensate the vertical alignment of markers and put at ground level to adopt navistack goals format
  tf::Transform tf(tf::createQuaternionFromYaw(tf::getYaw(base_marker_pose.pose.orientation) - M_PI/2.0),
                   tf::Vector3(base_marker_pose.pose.position.x, base_marker_pose.pose.position.y, 0.0));
  tf::StampedTransform marker_gb(tf, ros::Time::now(), global_frame_, "docking_base");

  // Check that we are not already close to the docking base
  if (tk::distance2D(robot_gb, marker_gb) < relay_on_marker_distance_)
  {
    ROS_DEBUG("Already close to the docking base (%.2f m), but we are not smart enough to make use of this... pabo io...",
              tk::distance2D(robot_gb, marker_gb));
  }

  // Half turn and translate to put goal at some distance in front of the marker
  tf::Transform in_front(tf::createQuaternionFromYaw(M_PI),
                         tf::Vector3(relay_on_marker_distance_/1.5, 0.0, 0.0));
  marker_gb *= in_front;

  move_base_msgs::MoveBaseGoal mb_goal;
  tk::tf2pose(marker_gb, mb_goal.target_pose.pose);
  mb_goal.target_pose.header.stamp = ros::Time::now();
  mb_goal.target_pose.header.frame_id = global_frame_;

  return dockInBase_(mb_goal);
}

bool Navigator::dockInBase_(const move_base_msgs::MoveBaseGoal& mb_goal)
{
  // Enable safety controller on normal navigation
  //enableSafety();

  // Wait for move base action servers to come up; the huge timeout is not a whim; move_base
  // action server can take up to 20 seconds to initialize in the official turtlebot laptop,
  // so is this request is issued short after startup...
  // WARN: move base server is not started unless global costmap has a positive update frequency
  if (waitForServer(move_base_ac_, 10.0) == false)
  {
    ROS_ERROR("Move base action server not available; we cannot navigate!");
    return cleanupAndError();
  }

  ROS_DEBUG("Sending goal to robot: %.2f, %.2f, %.2f (relative to %s)",
            mb_goal.target_pose.pose.position.x, mb_goal.target_pose.pose.position.y,
            tf::getYaw(mb_goal.target_pose.pose.orientation), mb_goal.target_pose.header.frame_id.c_str());
  goal_poses_pub_.publish(mb_goal.target_pose);
  sendGoal(mb_goal);
  state_ = GLOBAL_DOCKING;

  // Keep an eye open to detect the marker as we approach it
  if (ARMarkers::enableTracker() == false)
  {
    ROS_ERROR("Marker tracker not available; we cannot auto-dock!");
    return cleanupAndError();
  }

  ros::Time t0 = ros::Time::now();

  // Going to goal in front of the docking base marker
  while (move_base_ac_.waitForResult(ros::Duration(0.5)) == false)
  {
    if ((state_ == GLOBAL_DOCKING) &&
        ((ros::Time::now() - base_rel_pose_.header.stamp).toSec() < 1.0) &&
        (tk::distance2D(base_rel_pose_.pose) <= relay_on_marker_distance_))  // NOTE: base marker pose is relative to the robot
      {
      // Here is!
      ROS_INFO("Docking base spotted at %.2f m; switching to marker-based local navigation...",
               tk::distance2D(base_rel_pose_.pose));

      // Docking base marker spotted at relay_on_marker_distance; switch to relative goal
      if (cancelAllGoals(move_base_ac_) == false)
        ROS_WARN("Aish... we should not be here; nothing good is gonna happen...");

      // Get base marker tf on global reference system
      char marker_frame[32];
      sprintf(marker_frame, "ar_marker_%d", base_marker_id_);
      tf::StampedTransform marker_gb;

      try
      {
        tf_listener_.waitForTransform(global_frame_, marker_frame, base_rel_pose_.header.stamp, ros::Duration(10.5));
        tf_listener_.lookupTransform(global_frame_, marker_frame, base_rel_pose_.header.stamp, marker_gb);
      }
      catch (tf::TransformException& e)
      {
        // If this happens, I suppose it must be a bug in alvar tracker, as we use the msg timestamp
        ROS_ERROR("Cannot get tf %s -> %s: %s", base_frame_.c_str(), marker_frame, e.what());
        return cleanupAndError();
      }

      if (tk::roll(marker_gb) < -1.0)
      {
        // Sometimes markers are spotted "inverted" (pointing to -y); as we assume that all the markers are
        // aligned with y pointing up, x pointing right and z pointing to the observer, that's a recognition
        // error. We fix this flipping the tf.
        tf::Transform flip(tf::createQuaternionFromRPY(0.0, 0.0, M_PI));
        marker_gb *= flip;
      }

      // Compensate the vertical alignment of markers and put at ground level to adopt navistack goals format
      tf::Transform goal_gb(tf::createQuaternionFromYaw(tf::getYaw(marker_gb.getRotation()) - M_PI/2.0),
                            tf::Vector3(marker_gb.getOrigin().x(), marker_gb.getOrigin().y(), 0.0));

      // Project a pose relative to map frame in front of the docking base and heading to it
      // As before, half turn and translate to put goal at some distance in front of the marker
      tf::Transform in_front(tf::createQuaternionFromYaw(M_PI),
                             tf::Vector3(relay_on_beacon_distance_, 0.0, 0.0));
      goal_gb *= in_front;

      //< DEBUG
      tf::StampedTransform tf1(goal_gb, ros::Time::now(),  "map", "GOAL");
      tf::StampedTransform tf2(marker_gb, ros::Time::now(),  "map", "MARKER");
      tf_brcaster_.sendTransform(tf1);
      tf_brcaster_.sendTransform(tf2);
      //>

      // Send a second goal at relay_on_beacon_distance_ in front the marker
      move_base_msgs::MoveBaseGoal mb_goal_2;
      tk::tf2pose(goal_gb, mb_goal_2.target_pose.pose);

      mb_goal_2.target_pose.header.frame_id = global_frame_;
      mb_goal_2.target_pose.header.stamp = ros::Time::now();

      ROS_DEBUG("Sending goal to robot: %.2f, %.2f, %.2f (relative to %s)",
                mb_goal_2.target_pose.pose.position.x, mb_goal_2.target_pose.pose.position.y,
                tf::getYaw(mb_goal_2.target_pose.pose.orientation), mb_goal_2.target_pose.header.frame_id.c_str());
      goal_poses_pub_.publish(mb_goal_2.target_pose);
      sendGoal(mb_goal_2);

     state_ = MARKER_DOCKING;
    }
    else if ((ros::Time::now() - t0).toSec() < go_to_pose_timeout_)
    {
      ROS_DEBUG_THROTTLE(5.0, "Move base action state: %s (%.2f seconds elapsed)  %s",
                         move_base_ac_.getState().toString().c_str(),
                         (ros::Time::now() - t0).toSec(), state_ == GLOBAL_DOCKING?"GLOBAL_DOCKING":"MARKER_DOCKING");
    }
    else
    {
      // We reached go_to_pose_timeout_; given this timeout is generous, we must be lost or have another problem
      // TODO notify navigation watchdog and try to recover somehow
      ROS_WARN("Cannot she the docking base after %.2f seconds; current state is %s. Aborting...",
               go_to_pose_timeout_, move_base_ac_.getState().toString().c_str());
      return cleanupAndError();
    }
  }

  if (ARMarkers::disableTracker() == false)
  {
    ROS_WARN("Unable to stop AR markers tracker; we are spilling a lot of CPU!");
  }

  if (move_base_ac_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_WARN("Go to docking base failed: %s", move_base_ac_.getState().toString().c_str());
    return cleanupAndError();
  }

  if (state_ == GLOBAL_DOCKING)
  {
    ROS_WARN("Unable to spot docking base marker within the required distance");
    if (base_marker_id_ < AR_MARKERS_COUNT)
    {
      ROS_WARN("Last spot was %.2f seconds ago at %.2f meters",
               (ros::Time::now() - base_rel_pose_.header.stamp).toSec(), tk::distance2D(base_rel_pose_.pose));
    }
    // We cannot see the marker; that's normal if we are trying the odometry origin fallback solution (see
    // no-parameters dockInBase method). If not, probably we have a problem; switch on auto-docking anyway
  }

  // We should be in front of the docking base at base_beacon_distance_; switch on auto-docking

  // Wait for auto-docking action servers to come up (it should be already by now)
  if (waitForServer(auto_dock_ac_, 2.0) == false)
  {
    ROS_ERROR("Auto-docking action server not available; we cannot dock!");
    return cleanupAndError();
  }

  ROS_INFO("Switching on beacon-based auto-docking...");
  kobuki_msgs::AutoDockingGoal ad_goal;
  auto_dock_ac_.sendGoal(ad_goal);

  // Disable navigation safety controller on auto-docking to avoid bouncing against the base
  // It must be re-enabled on next wake-up
  disableSafety();

  bool retrying = false;
  t0 = ros::Time::now();

  while (auto_dock_ac_.waitForResult(ros::Duration(2.0)) == false)
  {
    if ((ros::Time::now() - t0).toSec() < auto_docking_timeout_)
    {
      ROS_DEBUG_THROTTLE(5.0, "Auto-dock action state: %s (%.2f seconds elapsed)",
                         auto_dock_ac_.getState().toString().c_str(), (ros::Time::now() - t0).toSec());
    }
    else if ((ros::Time::now() - t0).toSec() < auto_docking_timeout_*1.5)
    {
      if (retrying == false)
      {
        // Go back a bit and retry for an extra half timeout period  TODO really poor-man recovery...
        backward(0.20);
        ROS_WARN("Cannot auto-dock after %.2f seconds; current state is %s. Going backward to retry...",
                 (ros::Time::now() - t0).toSec(), auto_dock_ac_.getState().toString().c_str());
        retrying = true;
      }
      else
      {
        ROS_DEBUG_THROTTLE(5.0, "Auto-dock action state: %s (%.2f seconds elapsed)",
                           auto_dock_ac_.getState().toString().c_str(), (ros::Time::now() - t0).toSec());
      }
    }
    else
    {
      ROS_WARN("Cannot auto-dock after %.2f seconds; current state is %s. Aborting...",
               (ros::Time::now() - t0).toSec(), auto_dock_ac_.getState().toString().c_str());
      break;
      //enableSafety();
      //return cleanupAndError();
    }
  }

  if (auto_dock_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Successfully docked...  zzz...   zzz...");
    return cleanupAndSuccess("yawn.wav");
  }
  else
  {
    ROS_WARN("Go to docking base failed: %s", auto_dock_ac_.getState().toString().c_str());
    return cleanupAndError();
  }
}

bool Navigator::pickUpOrder(const geometry_msgs::PoseStamped& pickup_pose)
{
  // Enable safety controller on normal navigation
 // enableSafety();

  if (pickup_pose.header.frame_id != global_frame_)
  {
    ROS_ERROR("Pickup pose not in global frame (%s != %s)",
              pickup_pose.header.frame_id.c_str(), global_frame_.c_str());
    return cleanupAndError();
  }

  ROS_INFO("Global navigation to pickup point...");

  // Wait for move base action servers to come up; the huge timeout is not a whim; move_base
  // action server can take up to 20 seconds to initialize in the official turtlebot laptop,
  // so is this request is issued short after startup...
  // WARN: move base server is not started unless global costmap has a positive update frequency
  if (waitForServer(move_base_ac_, 10.0) == false)
  {
    ROS_ERROR("Move base action server not available; we cannot navigate!");
    return cleanupAndError();
  }

  int times_waiting = 0;
  double distance_to_pickup = std::numeric_limits<double>::infinity();
////  double heading_to_goal  = std::numeric_limits<double>::infinity();

  tf::StampedTransform robot_gb, pickup_gb;
  tk::pose2tf(pickup_pose, pickup_gb);

  move_base_msgs::MoveBaseGoal mb_goal;
  mb_goal.target_pose = pickup_pose;

  do
  {
    ROS_DEBUG("Sending goal to robot: %.2f, %.2f, %.2f (relative to %s)",
              mb_goal.target_pose.pose.position.x, mb_goal.target_pose.pose.position.y,
              tf::getYaw(mb_goal.target_pose.pose.orientation), mb_goal.target_pose.header.frame_id.c_str());
    goal_poses_pub_.publish(mb_goal.target_pose);
    sendGoal(mb_goal);
    state_ = GOING_TO_PICKUP;

    ros::Time t0 = ros::Time::now();

    // Going to pickup point
    while (move_base_ac_.waitForResult(ros::Duration(0.5)) == false)
    {
      if ((ros::Time::now() - t0).toSec() < go_to_pose_timeout_)
      {
        ROS_DEBUG_THROTTLE(5.0, "Move base action state: %s (%.2f seconds elapsed)", move_base_ac_.getState().toString().c_str(),
                           (ros::Time::now() - t0).toSec());

        if (recovery_behavior_ == true)
        {
          // Get latest robot global pose
          robot_gb = getRobotTf();

          // When close enough to the pickup pose, switch off recovery behavior so planner immediately fails if the
          // pickup point is busy; if not, robot will stupidly spin for a while before deciding that he must wait
          distance_to_pickup = tk::distance2D(robot_gb, pickup_gb);
          if (distance_to_pickup < close_to_pickup_distance_)
          {
            ROS_DEBUG("Close enough to the pickup point (%.2f < %.2f m); switch off recovery behavior",
                      distance_to_pickup, close_to_pickup_distance_);

            if (disableRecovery() == false)
            {
              ROS_WARN("Robot will stupidly spin for a while before deciding that pickup point is busy");
            }
          }
        }
      }
      else
      {
        ROS_WARN("Cannot reach pickup point after %.2f seconds; current state is %s. Aborting...",
                 go_to_pose_timeout_, move_base_ac_.getState().toString().c_str());
        return cleanupAndError();
      }
    }

    if (move_base_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("At pickup point! Waiting for 맛있는 커피...");
      return cleanupAndSuccess("pab.wav");
    }
    else
    {
      if (move_base_ac_.getState() == actionlib::SimpleClientGoalState::ABORTED)
      {
        times_waiting++;

        if ((recovery_behavior_ == true) && (distance_to_pickup > close_to_pickup_distance_*1.1))
        {
          // TODO/WARN the time lapses that I mention below are much bigger of what I claim because I'm happily
          //ignoring execution time, that is very big due to move_base planner; measure elapsed time instead

          // If this happen so early, something must be really wrong; anyway we will retry planning 3 times
          ROS_WARN("Move base aborted still at %.2f m from pickup point (we expect this to happen closer than %.2f m)",
                   distance_to_pickup, close_to_pickup_distance_);
          if (times_waiting > 3)
            return cleanupAndError();
        }
        else if (times_waiting > 15)
        {
          // So much waiting for pickup point... maybe something is wrong
          ROS_WARN("Pickup point don't get free after %.2f seconds... what the hell is happening?",
                   15*wait_for_pickup_point_);
          return cleanupAndError();
        }
        else
        {
          // Get latest robot global pose to calculate heading to pickup point
          robot_gb = getRobotTf();
          double to_turn = tk::wrapAngle(tk::heading(robot_gb, pickup_gb) - tf::getYaw(robot_gb.getRotation()));

          // Point toward the pickup point so we can see whether it gets free
          if (std::abs(to_turn) > 0.3)
            turn(to_turn);

          // We assume that the pickup point is busy; mooo, and wait for it to get free
          if (times_waiting % (int)std::ceil(10.0/wait_for_pickup_point_))
          {
            // Do not moo at more than 0.1 Hz... we don't want to be bothersome...
            ROS_INFO("Pickup point looks crowded... wait for %.2f seconds before retrying", wait_for_pickup_point_);
            if (play_sounds_) system(("rosrun waiterbot play_sound.bash " + resources_path_ + "/moo.wav").c_str());

            // Clear also the costmaps (at this point we have disabled recovery behaviors!)
            clearCostmaps();
          }
        }

        ros::Duration(wait_for_pickup_point_).sleep();
        continue;

        // TODO: I should try goals slightly displaced; or get the costmap and verify that the pickup AREA is really busy
      }
      else
      {
        // Something else (surely nasty) happen; just give up
        ROS_WARN("Go to pickup point failed: %s", move_base_ac_.getState().toString().c_str());
        return cleanupAndError();
      }
    }
  } while (true);
}

bool Navigator::deliverOrder(const geometry_msgs::PoseStamped& table_pose, double table_radius)
{
  // Enable safety controller on normal navigation
 // enableSafety();

  if (table_pose.header.frame_id != global_frame_)
  {
    ROS_ERROR("Table pose not in global frame (%s != %s)",
              table_pose.header.frame_id.c_str(), global_frame_.c_str());
    return cleanupAndError();
  }

  ROS_INFO("Global navigation to table...");

  // Wait for move base action servers to come up; at this point the should be up already
  if (waitForServer(move_base_ac_, 2.0) == false)
  {
    ROS_ERROR("Move base action server not available; we cannot navigate!");
    return cleanupAndError();
  }

  int attempts = 0;
  double min_tried_heading = std::numeric_limits<double>::max();
  double max_tried_heading = std::numeric_limits<double>::min();
  double heading_increment = std::atan2(2.0*robot_radius_, table_radius + tables_serving_distance_);

  tf::StampedTransform table_gb;
  tk::pose2tf(table_pose, table_gb);

  do
  {
    // Get latest robot global pose to calculate heading and distance from robot to table
    tf::StampedTransform robot_gb = getRobotTf();

    double distance_to_table, last_plan_distance_to_table;
    double heading_to_table,  last_plan_heading_to_table;
    distance_to_table = last_plan_distance_to_table = tk::distance2D(robot_gb, table_gb);
    heading_to_table  = last_plan_heading_to_table  = tk::heading(robot_gb, table_gb);

    // Heading to goal is the same that to table for the first try, but it drifts when we fail to reach a goal
    double heading_to_goal = heading_to_table;

    if ((heading_to_table >= min_tried_heading) && (heading_to_table <= max_tried_heading))
    {
      if (std::abs(heading_to_table - min_tried_heading) < std::abs(heading_to_table - max_tried_heading))
      {
        heading_to_goal = min_tried_heading - heading_increment;
      }
      else
      {
        heading_to_goal = max_tried_heading + heading_increment;
      }
    }

    // Table tf in global reference system; we set robot-to-table heading as orientation...
    table_gb.setRotation(tf::createQuaternionFromYaw(tk::wrapAngle(heading_to_goal)));

    // ...and displace table radius + robot radius + safety margin in that direction
    tf::Transform towards_robot(tf::Quaternion::getIdentity(),
                                tf::Vector3(- table_radius - tables_serving_distance_, 0.0, 0.0));
    tf::StampedTransform goal_gb(table_gb*towards_robot, ros::Time::now(), global_frame_, "DELIVERY_POINT");
    double distance_to_goal = tk::distance2D(robot_gb, goal_gb);

    //< DEBUG
    tf::StampedTransform table(table_gb, ros::Time::now(),  global_frame_, "TABLE");
    tf_brcaster_.sendTransform(table);
    tf_brcaster_.sendTransform(goal_gb);
    //>

    move_base_msgs::MoveBaseGoal mb_goal;
    tk::tf2pose(goal_gb, mb_goal.target_pose);

    ROS_DEBUG("Sending goal to robot: %.2f, %.2f, %.2f (relative to %s)",
              mb_goal.target_pose.pose.position.x, mb_goal.target_pose.pose.position.y,
              tf::getYaw(mb_goal.target_pose.pose.orientation), mb_goal.target_pose.header.frame_id.c_str());
    goal_poses_pub_.publish(mb_goal.target_pose);
    sendGoal(mb_goal);
    state_ = GOING_TO_TABLE;

    ros::Time t0 = ros::Time::now();

    // Going to delivery point
    while (move_base_ac_.waitForResult(ros::Duration(0.5)) == false)
    {
      if ((ros::Time::now() - t0).toSec() < go_to_pose_timeout_)
      {
        ROS_DEBUG_THROTTLE(5.0, "Move base action state: %s (%.2f seconds elapsed)", move_base_ac_.getState().toString().c_str(),
                           (ros::Time::now() - t0).toSec());

        // Get latest robot global pose to calculate heading and distance from robot to table and goal (delivery point)
        robot_gb = getRobotTf();
        distance_to_table = tk::distance2D(robot_gb, table_gb);
        heading_to_table  = tk::heading(robot_gb, table_gb);

        distance_to_goal  = tk::distance2D(robot_gb, goal_gb);

        if (distance_to_table < (table_radius + tables_serving_distance_ + 0.1))
        {
          // Somehow we manage to approach the table, so... why to bother more? Just cancel goal and head to the table center
          if (cancelAllGoals(move_base_ac_) == false)
            ROS_WARN("Aish... we should not be here; nothing good is gonna happen...");

          double to_turn = tk::wrapAngle(heading_to_table - tf::getYaw(robot_gb.getRotation()));
          ROS_DEBUG("Already close to the table while going to next goal (%.2f m); just turn %.2f rad to face the table",
                    distance_to_table, to_turn);
          if (std::abs(to_turn) > 0.3)
            turn(to_turn);

          return cleanupAndSuccess("kaku.wav");
        }
        else if (distance_to_goal < close_to_delivery_distance_)
        {
          // When close enough to the table, switch off recovery behavior so planner immediately fails if the
          // desired delivery point is busy; if not, robot will spin instead of looking for a different point
          if (recovery_behavior_ == true)
          {
            ROS_DEBUG("Close enough to the delivery point (%.2f < %.2f m); switch off recovery behavior",
                      distance_to_goal, close_to_delivery_distance_);

            if (disableRecovery() == false)
            {
              ROS_WARN("Robot will stupidly spin for a while before deciding that a delivery point is busy");
            }
          }

          // We also start checking our heading to the table, so if it changes a lot, and we are not very close,
          // we replan our approaching point to the table
          // Note: we can cancel goal only when recovery behavior is disabled; if not it takes ages to work!
          if ((std::abs(last_plan_heading_to_table - heading_to_table) > 0.5) &&
              (distance_to_table > 3*table_radius) && (recovery_behavior_ == false))
          {
            ROS_DEBUG("Heading to the table has notably changed (%.2f -> %.2f m); replan approach point",
                      last_plan_heading_to_table, heading_to_table);
            if (cancelAllGoals(move_base_ac_) == false)
              ROS_WARN("Aish... we should not be here; nothing good is gonna happen...");  // TODO do I really need to cancel?  cannot just preempt?
            break;
          }
        }
      }
      else
      {
        // Go to pose timeout... assuming it's long enough, we must be really lost; TODO notify nav watchdog
        ROS_WARN("Cannot reach delivery point after %.2f seconds; current state is %s. Aborting...",
                 go_to_pose_timeout_, move_base_ac_.getState().toString().c_str());
        return cleanupAndError();
      }
    }

    ROS_DEBUG("KKKK  %s", move_base_ac_.getState().getText().c_str());

    // Ok, we left the wait for goal loop; let's see what happened

    if (move_base_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("At delivery point! Waiting for user confirmation...");
      return cleanupAndSuccess("kaku.wav");
    }
    else if (move_base_ac_.getState() == actionlib::SimpleClientGoalState::PREEMPTED)
    {
      // Previous goal has been cancelled around 30 lines before
      ROS_DEBUG("Move base action preempted so we are replaning our approaching point to the table");
    }
    else if (move_base_ac_.getState() == actionlib::SimpleClientGoalState::ABORTED)
    {
      attempts++;

      if ((recovery_behavior_ == true) && (distance_to_goal > close_to_delivery_distance_*1.1))
      {
        // If this happen so early, something must be really wrong; anyway we will retry planning 3 times
        ROS_WARN("Move base aborted still at %.2f m from delivery point (we expect this to happen closer than %.2f m)",
                 distance_to_goal, close_to_delivery_distance_);
        if (attempts > 3)
          return cleanupAndError();
      }
      else if (( max_tried_heading - min_tried_heading) > (2.0*M_PI - heading_increment))
      {
        // Busy sector surrounds the table! use our crappy delivery fallback;  maybe increase tables_serving_distance_ and retry???  TODO-OOOOOOOOOOOOOOOOO!!!!
        ROS_INFO("All delivery points looks busy (%d attempts). Just stand and cry...", attempts);
        return cleanupAndSuccess("kaku.wav");
      }
      else
      {
        ROS_WARN("Delivery point looks busy; try another one (%d attempts)", attempts);

        // Delivery point looks busy; increase the already tried sector so the planner can choose a new delivery point
        // Note that we wrap busy sector thresholds from -2*pi to +2*pi to deal with the -pi/+pi singularity
        min_tried_heading = std::min(min_tried_heading, heading_to_goal - heading_increment);
        max_tried_heading = std::max(max_tried_heading, heading_to_goal + heading_increment);

        // Clear also the costmaps (at this point we have disabled recovery behaviors!)
        clearCostmaps();

//        ROS_DEBUG("heading_to_goal: %.2f, %.2f, %.2f      %.2f           %.2f     %d", heading_to_goal, min_tried_heading ,max_tried_heading,
//                     heading_increment ,  ( max_tried_heading - min_tried_heading),  (( max_tried_heading - min_tried_heading) >= 2.0*M_PI));
      }
    }
    else
    {
      // Something else (surely nasty) happen; just give up
      ROS_WARN("Unexpected goal state: %s. Go to delivery point failed", move_base_ac_.getState().toString().c_str());
      return cleanupAndError();
    }
  } while (true);
}

bool Navigator::cleanupAndSuccess(const std::string& wav_file)
{
  if ((wav_file.length() > 0) && (play_sounds_))
    system(("rosrun waiterbot play_sound.bash " + resources_path_ + wav_file).c_str());

  // Revert to standard configuration after completing a task
  //  - clear costmaps, mostly to restore global map to source bitmap
  //  - (re)enable safety controller for normal operation
  //  - disable AR markers tracker as it's a CPU spendthrift
  clearCostmaps();
 // disableSafety();
  enableRecovery();

//  cancelAllGoals(move_base_ac_);//, recovery_behavior_ == true ? 10.0 : 2.0);
//  cancelAllGoals(auto_dock_ac_);
//  ARMarkers::disableTracker();
  state_ = IDLE;

  return true;
}

bool Navigator::cleanupAndError()
{
  // Something went wrong in one of the chaotic methods of this class; try at least the let all properly
  // WARN1 cancel move base goals fails if it's executing recovery behavior  TODO: how to deal with this?
  // WARN2 we are very radical on this method, restoring things that probably have not being used... be careful!
  clearCostmaps();
//  disableSafety();
  enableRecovery();
  cancelAllGoals(move_base_ac_);
  cancelAllGoals(auto_dock_ac_);
  ARMarkers::disableTracker();
  state_ = IDLE;

  return false;
}


bool Navigator::moveBaseReset()
{
  // Something went wrong in one of the chaotic methods of this class; try at least the let all properly
  // WARN1 cancel move base goals fails if it's executing recovery behavior  TODO: how to deal with this?
  // WARN2 we are very radical on this method, restoring things that probably have not being used... be careful!
  int i = 0;

  ROS_WARN("Canceling goal with %s state...", move_base_ac_.getState().toString().c_str());
//  PENDING,
//  ACTIVE,
//  RECALLED,
//  REJECTED,
//  PREEMPTED,
//  ABORTED,
//  SUCCEEDED,
//  LOST

  move_base_ac_.cancelGoal();
  while (move_base_ac_.waitForResult(ros::Duration(0.1)) == false)
  {

    if (i == 10)
    {
      ROS_WARN("Cancel goal didn't finish after %.2f seconds: %s  Try disabling recovery", 0.1, move_base_ac_.getState().toString().c_str());
      ROS_DEBUG("%d", disableRecovery());
    }
    else{
      ROS_WARN("Cancel goal didn't finish after %.2f seconds: %s", 0.1, move_base_ac_.getState().toString().c_str());
      move_base_ac_.cancelGoal();

    }
    ros::Duration(1.0).sleep();

    i++;
  }
  ROS_DEBUG("%d", enableRecovery());

  return true;
}


bool Navigator::enableRecovery()
{
  if (recovery_behavior_ == true)
    return true;

  ros::Time t0 = ros::Time::now();
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<dynamic_reconfigure::Reconfigure>("move_base/set_parameters");
  dynamic_reconfigure::Reconfigure srv;
  srv.request.config.bools.resize(1);
  srv.request.config.bools[0].name = "recovery_behavior_enabled";
  srv.request.config.bools[0].value = true;
//  srv.request.config.doubles.resize(1);
//  srv.request.config.doubles[0].name = "planner_frequency";
//  srv.request.config.doubles[0].value = default_planner_frequency_;

  if (client.call(srv) == true)
  {
    ROS_INFO("Recovery behavior enabled (%f seconds)", (ros::Time::now() - t0).toSec());
    recovery_behavior_ = true;
    return true;
  }
  else
  {
    ROS_ERROR("Failed to enable recovery behavior (%f seconds)", (ros::Time::now() - t0).toSec());
    return false;
  }



  int status = system("rosrun dynamic_reconfigure dynparam set move_base " \
                       "\"{ recovery_behavior_enabled: true }\"");  // clearing_rotation_allowed
  if (status != 0)
  {
    ROS_ERROR("Enable recovery behavior failed (%d/%d)", status, WEXITSTATUS(status));
    return false;
  }
ROS_DEBUG("enableRecovery ended on %f s", (ros::Time::now() - t0).toSec());
  recovery_behavior_ = true;
  return true;
}

bool Navigator::disableRecovery()
{
  if (recovery_behavior_ == false)
    return true;

  ros::Time t0 = ros::Time::now();
  ROS_DEBUG("disableRecovery starts....");

  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<dynamic_reconfigure::Reconfigure>("move_base/set_parameters");
  dynamic_reconfigure::Reconfigure srv;
  srv.request.config.bools.resize(1);
  srv.request.config.bools[0].name = "recovery_behavior_enabled";
  srv.request.config.bools[0].value = false;
//  srv.request.config.doubles.resize(1);
//  srv.request.config.doubles[0].name = "planner_frequency";
//  srv.request.config.doubles[0].value = 0.0;

  if (client.call(srv) == true)
  {
    ROS_INFO("Recovery behavior disabled (%f seconds)", (ros::Time::now() - t0).toSec());
    recovery_behavior_ = false;
    return true;
  }
  else
  {
    ROS_ERROR("Failed to disable recovery behavior (%f seconds)", (ros::Time::now() - t0).toSec());
    return false;
  }


  int status = system("rosrun dynamic_reconfigure dynparam set move_base " \
                       "\"{ recovery_behavior_enabled: false }\"");  // clearing_rotation_allowed
  if (status != 0)
  {
    ROS_ERROR("Disable recovery behavior failed (%d/%d)", status, WEXITSTATUS(status));
    return false;
  }
ROS_DEBUG("disableRecovery ended on %f", (ros::Time::now() - t0).toSec());
  recovery_behavior_ = false;
  return true;
}

bool Navigator::clearCostmaps()
{
  ros::Time t0 = ros::Time::now();

  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("move_base/clear_costmaps");
  std_srvs::Empty srv;

  if (client.call(srv) == true)
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

bool Navigator::shoftRecovery()
{
  if (recovery_behavior_ == true)
    return true;

  int status = system("rosrun dynamic_reconfigure dynparam set move_base " \
                       "\"{ recovery_behaviors: [{name: conservative_reset, type: clear_costmap_recovery/ClearCostmapRecovery}] }\"");  // recovery_behavior_enabled
  if (status != 0)
  {
    ROS_ERROR("Enable recovery behavior failed (%d/%d)", status, WEXITSTATUS(status));
    return false;
  }

  recovery_behavior_ = true;
  return true;
}

bool Navigator::hardRecovery()
{
  if (recovery_behavior_ == false)
    return true;

  int status = system("rosrun dynamic_reconfigure dynparam set move_base " \
                       "\"{ recovery_behaviors: [{name: conservative_reset, type: clear_costmap_recovery/ClearCostmapRecovery}, {name: rotate_recovery, type: rotate_recovery/RotateRecovery}, {name: aggressive_reset, type: clear_costmap_recovery/ClearCostmapRecovery}] }\"");  // recovery_behavior_enabled
  if (status != 0)
  {
    ROS_ERROR("Disable recovery behavior failed (%d/%d)", status, WEXITSTATUS(status));
    return false;
  }

  recovery_behavior_ = false;
  return true;
}

void Navigator::enableMotors()
{
  kobuki_msgs::MotorPower msg;
  msg.state = kobuki_msgs::MotorPower::ON;
  motor_pwr_pub_.publish(msg);
}

void Navigator::disableMotors()
{
  kobuki_msgs::MotorPower msg;
  msg.state = kobuki_msgs::MotorPower::OFF;
  motor_pwr_pub_.publish(msg);
}

tf::StampedTransform Navigator::getTf(const std::string& frame_1, const std::string& frame_2)
{
  // Use this just to get tf that cannot fail unless some part of the localization chain is missing
  // Otherwise said; if this exception happens we are really pissed-off, so we don't try to recover
  tf::StampedTransform tf;
  try
  {
    tf_listener_.lookupTransform(frame_1, frame_2, ros::Time(0.0), tf);
  }
  catch (tf::TransformException& e)
  {
    ROS_WARN("Cannot get tf %s -> %s: %s", frame_1.c_str(), frame_1.c_str(), e.what());
  }
  return tf;
}

tf::StampedTransform Navigator::getOdomTf()
{
  // Get latest map -> odom tf
  return getTf(global_frame_, odom_frame_);
}

tf::StampedTransform Navigator::getRobotTf()
{
  // Get latest robot global pose
  return getTf(global_frame_, base_frame_);
}

} /* namespace waiterbot */
