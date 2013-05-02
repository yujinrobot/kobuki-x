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
    while(!initialized_table_)
    {
      ROS_DEBUG("Table is not initialized yet...");
      ros::Duration(1).sleep();
    }                                                         
  }

  bool WaiterNode::getReadyToWork()
  {
    if(initialized_) { 
      return leaveNest();    
    }
    else {
      return wakeUp();
    }
  }
  
  bool WaiterNode::waitForButton()
  {
    return true;
  }
  
  bool WaiterNode::wakeUp()
  {
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
    while ((ar_markers_.spotted(1.0, 10, true, spotted_markers) == false) && (timeout == false))
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
    }
  
    ar_track_alvar::AlvarMarker closest_marker;
    ar_markers_.closest(1.0, 10, true, closest_marker);
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
    t0 = ros::Time::now();
    while ((ar_markers_.spotDockMarker(base_marker_id) == false) && (timeout == false))
    {
      navigator_.turnClockwise();
      if ((ros::Time::now() - t0).toSec() >= SPOT_BASE_MARKER_TIMEOUT)
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
  
    // Now... we are ready to go!   -> go to kitchen
    ROS_INFO("Waking up successfully completed in %.2f seconds; ready to go!", (ros::Time::now() - t0).toSec());
  
    return cleanupAndSuccess();
  
    //chijon  ->  plot MARKERS global
  
    /*
    vel.angular.z = 0.0;
    cmd_vel_pub_.publish(vel);
  
    timeout = false;
    t0 = ros::Time::now();
    while ((ar_markers_.spotted(1.0, global_markers_, ar_track_alvar::AlvarMarkers(), spotted_markers) == false) &&
           (timeout == false))
    {
      ros::Duration(0.1).sleep();
      if ((ros::Time::now() - t0).toSec() >= SPOT_POSE_MARKER_TIMEOUT)
      {
        timeout = true;
      }
    }
  
    // stop after moving a bit more (TODO move until we have the marker in front)
    ros::Duration(1.0).sleep();
    vel.angular.z = 0.0;
    cmd_vel_pub_.publish(vel);
  
    if (timeout == true)
    {
      ROS_WARN("Unable to detect globally localized AR marker; aborting wake up!");
      return;
  
      // TODO do nothing more by now, but we will want to make an error sound, red leds, etc.
    }
  
    ar_markers_.closest(spotted_markers, ar_track_alvar::AlvarMarkers(), closest_marker);
    ROS_DEBUG("Global localization AR marker %d spotted! Initialize amcl...", closest_marker.id);
  
  
    // Check the localization watchdog; we should be localized now
  
    // Complete a loop looking again for OUR docking base marker; it should be more or less in front
    vel.angular.z = -0.5;
    cmd_vel_pub_.publish(vel);
  //  ros::Duration(5.0).sleep();
    while (tf::getYaw(odometry_.pose.pose.orientation) > 0.0)  ros::Duration(0.1).sleep();
    while (tf::getYaw(odometry_.pose.pose.orientation) < 0.0)  ros::Duration(0.1).sleep();
  
    vel.angular.z = 0.0;
    cmd_vel_pub_.publish(vel);
  */
  }

  bool WaiterNode::leaveNest()                                                                
  {
    ROS_DEBUG("Leaving the nest when already localized; slowly moving back for half meter");
    navigator_.backward(0.5);
    return true;
  }

  bool WaiterNode::cleanupAndSuccess()
  {
    // Revert to standard configuration after completing a task
    //  - (re)enable safety controller for normal operation
    //  - (re)enable motors (auto-docking disables them after finishing)
    //  - disable AR markers tracker as it's a CPU spendthrift
    navigator_.enableMotors();
    navigator_.enableSafety();
    ar_markers_.disableTracker();

    return true;
  }

  bool WaiterNode::cleanupAndError()
  {
    // Something went wrong in one of the chaotic methods of this class; try at least the let all properly
    navigator_.enableMotors();
    navigator_.enableSafety();
    ar_markers_.disableTracker();

    return false;
  }
}


