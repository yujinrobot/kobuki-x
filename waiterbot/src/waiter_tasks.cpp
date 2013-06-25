/*
 * waiter_tasks.cpp
 *
 *  Created on: Apr 6, 2013
 *      Author: jorge
 */

#include "waiterbot/waiter_node.hpp"

namespace waiterbot
{

bool WaiterNode::waitForPoses()
{
  // check whether semantic poses are initialized
  while (!initialized_table_)
  {
    ROS_DEBUG("Table is not initialized yet...");
    ros::Duration(1).sleep();
  }
}

bool WaiterNode::getReadyToWork()
{
  if (initialized_)
  {
    return leaveNest();
  }
  else
  {
    return wakeUp();
  }
}

bool WaiterNode::waitForButton()
{
  ROS_INFO("Waiting for button...");

  if (debug_mode_ == true)
  {
    ros::Duration(3.0).sleep();
    ROS_INFO("Button faked with a 3.0 seconds timeout");
    return true;
  }

  wait_for_button_ = true;
  while (ros::ok() && wait_for_button_)
  {
    ros::Duration(0.5).sleep();
  }

  ROS_INFO("Button pressed");
  return true;
}

bool WaiterNode::wakeUp()
{
  // TODO: unlike on the rest of the app. we make calla to ar_markers_ instead of registering callbacks!
  // (in fact, ar_markers_ should be a node). Also this is very brittle; we have no fallback mechanisms at all

  ROS_DEBUG("Waking up! first try to recognize our own nest; slowly moving back...");

  if (ar_markers_.enableTracker() == false)
  {
    ROS_ERROR("Unable to start AR markers tracker; aborting wake up!");
    return cleanupAndError();
  }

  // Move back until we detect the AR marker identifying this robot's docking station
  bool timeout = false;
  ros::Time t0 = ros::Time::now();
  ar_track_alvar::AlvarMarkers spotted_markers;
  while ((ar_markers_.spotted(1.0, 0.3, true, spotted_markers) == false) && (timeout == false))
  {
    if ((ros::Time::now() - t0).toSec() < SPOT_BASE_MARKER_TIMEOUT)
    {
      navigator_.slowBackward();
    }
    else if ((ros::Time::now() - t0).toSec() < SPOT_BASE_MARKER_TIMEOUT + 2.0)
    {
      // Wait stopped an extra couple of seconds before giving up
      navigator_.stop();
    }
    else
    {
      timeout = true;
    }
  }

  navigator_.stop();

  if (timeout == true)
  {
    ROS_WARN("Unable to detect docking station AR marker; aborting wake up!");
    return cleanupAndError();

    // TODO do nothing more by now, but we will want to make an error sound, red leds, etc.
    // Possible fallback: assume that odom + robot radius + docking base width is its pose
  }

  ar_track_alvar::AlvarMarker closest_marker;
  ar_markers_.closest(1.0, 0.3, true, closest_marker);
  ROS_DEBUG("Docking station AR marker %d spotted! Look for a global marker to find where I am...", closest_marker.id);

  uint32_t base_marker_id = closest_marker.id;
  //base_marker_.header.frame_id = "map";

  // Now look for a global marker to initialize our localization; full spin clockwise
  navigator_.spinClockwise();

  // After a full spin, we should be localized and in front of our docking base marker
  if (nav_watchd_.localized() == false)
  {
    // Something went wrong... we should have a fall-back mechanism, but this is TODO
    ROS_WARN("Still not localized! Probably we cannot see a globally localized marker");
    return cleanupAndError();
  }

  ROS_DEBUG("We are now localized; look again for our docking station marker...");

  // Look (again) for our docking station marker; should be just in front of us!
  timeout = false;
  ros::Time t1 = ros::Time::now();
  while ((ar_markers_.spotDockMarker(base_marker_id) == false) && (timeout == false))
  {
    navigator_.turnClockwise();
    if ((ros::Time::now() - t1).toSec() >= SPOT_BASE_MARKER_TIMEOUT)
    {
      timeout = true;
    }
  }

  navigator_.stop();

  if (timeout == true)
  {
    // Nope! again something went wrong... and again the fall-back mechanism is TODO
    ROS_WARN("Unable to detect docking station AR marker; aborting wake up!");
    return cleanupAndError();

    // TODO do nothing more by now, but we will want to make an error sound, red leds, etc.
  }

  ROS_DEBUG("Docking station AR marker %d spotted and registered as a global marker", base_marker_id);

  if (ar_markers_.disableTracker() == false)
  {
    ROS_WARN("Unable to stop AR markers tracker; we are spilling a lot of CPU!");
  }

  // Now... we are ready to go!
  ROS_INFO("Waking up successfully completed in %.2f seconds; ready to go!", (ros::Time::now() - t0).toSec());
  initialized_ = true;

  // Reset costmaps, as they surely contains a lot of errors due to the wrong initial localization
  navigator_.clearCostmaps();

  // Enable safety controller for normal navigation
  navigator_.enableSafety();

  return cleanupAndSuccess();
}

bool WaiterNode::leaveNest()
{
  ROS_DEBUG("Leaving the nest when already localized; slowly moving back for half meter");
  navigator_.backward(0.5);

  // Enable safety controller for normal navigation
  navigator_.enableSafety();

  return true;
}

bool WaiterNode::cleanupAndSuccess()
{
  // Revert to standard configuration after completing a task
  //  - (re)enable safety controller for normal operation
  //  - disable AR markers tracker as it's a CPU spendthrift
//  navigator_.enableSafety();
  ar_markers_.disableTracker();

  return true;
}

bool WaiterNode::cleanupAndError()
{
  // Something went wrong in one of the chaotic methods of this class; try at least the let all properly
//  navigator_.enableSafety();
  ar_markers_.disableTracker();

  return false;
}

bool WaiterNode::gotoTable(int table_id)
{
  // find table pose
  bool table_found = false;
  float radius;
  geometry_msgs::PoseStamped table_pose;
  for (unsigned int i = 0; i < table_poses_.tables.size(); i++)
  {
    // Look for the requested table's pose (and get rid of the useless covariance)
    if (table_poses_.tables[i].name.find(tk::nb2str(table_id), strlen("table")) != std::string::npos)
    {
      ROS_DEBUG("Target table %d: rad = %f, pose = %s", table_id, table_poses_.tables[i].radius,
                tk::pose2str(table_poses_.tables[i].pose_cov_stamped.pose.pose));
      table_pose.header = table_poses_.tables[i].pose_cov_stamped.header;
      table_pose.pose = table_poses_.tables[i].pose_cov_stamped.pose.pose;
      radius = table_poses_.tables[i].radius;
      table_found = true;
      break;
    }
  }

  if (table_found == false)
  {
    ROS_WARN("Table %d not found! bloody jihoon...  ignoring order", table_id);
    return false;
  }
  else
  {
    return navigator_.deliverOrder(table_pose, radius);
  }
}

} // namespace waiterbot
