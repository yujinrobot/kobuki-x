/*
 * waiter_node.cpp
 *
 *  Created on: Apr 6, 2013
 *      Author: jorge
 */

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/Led.h>
#include <kobuki_msgs/Sound.h>

#include "waiterbot/waiter_node.hpp"

namespace waiterbot
{

bool WaiterNode::init()
{
  // register the goal and feeback callbacks
  as_.registerGoalCallback(boost::bind(&WaiterNode::deliverOrderCB, this));
  as_.registerPreemptCallback(boost::bind(&WaiterNode::preemptOrderCB, this));
  as_.start();

//  deliver_as_  = nh_., name, boost::bind(&WaiterNode::deliverOrderCB, this, _1), false),

  led_1_pub_    = nh_.advertise <kobuki_msgs::Led>     ("mobile_base/commands/led1", 1);
  led_2_pub_    = nh_.advertise <kobuki_msgs::Led>     ("mobile_base/commands/led2", 1);
  sound_pub_    = nh_.advertise <kobuki_msgs::Sound>   ("mobile_base/commands/sound", 1);

  core_sensors_sub_ = nh_.subscribe("core_sensors",    5, &WaiterNode::coreSensorsCB, this);
  table_poses_sub_  = nh_.subscribe("table_pose_list", 1, &WaiterNode::tablePosesCB, this);

  // Initialize sub-modules
  if (nav_watchd_.init() == false)
  {
    ROS_ERROR("Navigation watchdog initialization failed; shutting down %s node", node_name_.c_str());
    return false;
  }
  if (ar_markers_.init() == false)
  {
    ROS_ERROR("AR markers initialization failed; shutting down %s node", node_name_.c_str());
    return false;
  }
  if (navigator_.init() == false)
  {
    ROS_ERROR("Navigator initialization failed; shutting down %s node", node_name_.c_str());
    return false;
  }

  // Configure call-backs between submodules
  ar_markers_.setRobotPoseCB(boost::bind(&NavWatchdog::arMarkerMsgCB, &nav_watchd_, _1));
  ar_markers_.baseSpottedCB(boost::bind(&Navigator::baseSpottedMsgCB, &navigator_, _1, _2));

  // Enable safety controller by default
  navigator_.enableSafety();

  return true;
}

void WaiterNode::tablePosesCB(const semantic_region_handler::TablePoseList::ConstPtr& msg)
{
  // Just take first message; ignore the rest, as global markers list is not dynamic
  if ((table_poses_.tables.size() == 0) && (msg->tables.size() > 0))
  {
    table_poses_ = *msg;
    ROS_INFO("%lu table pose(s) received", table_poses_.tables.size());
    for (unsigned int i = 0; i < table_poses_.tables.size(); i++)
    {
      // Look for the pickup point
      if (table_poses_.tables[i].name.find("pickup") != std::string::npos)
      {
        ROS_DEBUG("Pickup point: %s", tk::pose2str(table_poses_.tables[i].pose_cov_stamped.pose.pose));
        pickup_pose_.header = table_poses_.tables[i].pose_cov_stamped.header;
        pickup_pose_.pose = table_poses_.tables[i].pose_cov_stamped.pose.pose;
      }
      else
      {
        ROS_DEBUG("%s. rad: %f, pose: %s", table_poses_.tables[i].name.c_str(), table_poses_.tables[i].radius,
                  tk::pose2str(table_poses_.tables[i].pose_cov_stamped.pose.pose));
      }
    }
  }
}

void WaiterNode::deliverOrderCB()
{
  //  if (status_ == busy o jodido )   reject

  // accept the new goal
  order_ = as_.acceptNewGoal()->order;
  ROS_INFO("Deliver order action requested [order: %d, table: %d]", order_.order_id, order_.table_id);

  if (order_.order_id == 0)
//  if (status_ == robot IDLE)
//  {
    boost::thread wakeUpThread(&WaiterNode::wakeUp, this);
//  }
  //order_status_ = cafe_msgs::Status::IDLE;//TODO ojo; mato la thread????

  if (order_.order_id == 1)
    boost::thread wakeUpThread(&WaiterNode::leaveNest, this);

  if (order_.order_id == 2)
  {
//    if (ar_markers_.enableTracker() == false)
//    {
//      ROS_ERROR("Unable to start AR markers tracker; aborting wake up!");
//      return;
//    }
    boost::thread dockingThread(&Navigator::dockInBase, &navigator_, ar_markers_.getDockingBasePose());

  // TODO  ar_markers_.disableTracker()  >>> cuando decida la logica, decido donde y cuando enable disable AR track
    ///>>>>  creo que mejor en el FSM , xq las task pueden abortar por muuuuuchas cosas,  y habria q poner un disable en cada una

  }

  if (order_.order_id == 3)
    boost::thread pickUpThread(&Navigator::pickUpOrder, &navigator_, pickup_pose_);

  if (order_.order_id == 4)
  {
    bool table_found = false;
    for (unsigned int i = 0; i < table_poses_.tables.size(); i++)
    {
      // Look for the requested table's pose (and get rid of the useless covariance)
      if (table_poses_.tables[i].name.find(tk::nb2str(order_.table_id), strlen("table")) != std::string::npos)
      {
        ROS_DEBUG("Target table %d: rad = %f, pose = %s", order_.table_id, table_poses_.tables[i].radius,
                  tk::pose2str(table_poses_.tables[i].pose_cov_stamped.pose.pose));
        geometry_msgs::PoseStamped table_pose;
        table_pose.header = table_poses_.tables[i].pose_cov_stamped.header;
        table_pose.pose = table_poses_.tables[i].pose_cov_stamped.pose.pose;

        boost::thread pickUpThread(&Navigator::deliverOrder, &navigator_,
                                   table_pose, table_poses_.tables[i].radius);
        table_found = true;
        break;
      }
    }

    if (table_found == false)
    {
      ROS_WARN("Table %d not found! bloody jihoon...  ignoring order", order_.table_id);
    }
  }


  if (order_.order_id == 5)
  {
    boost::thread kk(&Navigator::moveBaseReset, &navigator_);
  }


  if (order_.order_id == 6)
    boost::thread kk(&Navigator::turn, &navigator_, -M_PI*(3.0/4.0));
  if (order_.order_id == 7)
    boost::thread kk(&Navigator::turn, &navigator_, -M_PI*2);
  if (order_.order_id == 8)
    boost::thread kk(&Navigator::turn, &navigator_, M_PI*1.5);
  if (order_.order_id == 9)
    boost::thread kk(&Navigator::turn, &navigator_, -M_PI*0.5);

  if (order_.order_id == 10)
  {
    if (ar_markers_.enableTracker() == false)
    {
      ROS_WARN("Unable to start AR markers tracker");
    }
  }

  if (order_.order_id == 11)
  {
    if (ar_markers_.disableTracker() == false)
    {
      ROS_WARN("Unable to stop AR markers tracker; we are spilling a lot of CPU!");
    }
  }


// TODO todo esto al control loop... (cuando lo tenga)
  // publish the feedback
  feedback_.status = cafe_msgs::Status::IDLE;//order_status_;
  as_.publishFeedback(feedback_);

  result_.result = "YA VEREMOS...";
  as_.setSucceeded(result_);

//  if(success)
//  {
//    result_.sequence = feedback_.sequence;
//    ROS_INFO("%s: Succeeded", action_name_.c_str());
//    // set the action state to succeeded
//    as_.setSucceeded(result_);
//  }
}
void WaiterNode::preemptOrderCB()
{
  ROS_WARN("Current order preempted [order: %d, table: %d]", order_.order_id, order_.table_id);
  // set the action state to preempted
  //  TODO WE REALLY WANT???  as_.setPreempted();
}

void WaiterNode::coreSensorsCB(const kobuki_msgs::SensorState::ConstPtr& msg)
{
  core_sensors_ = *msg;
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

void WaiterNode::spin()
{
  ros::Rate rate(50.0);

  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
}

} /* namespace waiterbot */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waiterbot");

  waiterbot::WaiterNode node(ros::this_node::getName());
  if (node.init() == false)
  {
    ROS_ERROR("%s initialization failed", ros::this_node::getName().c_str());
    return -1;
  }

  node.spin();

  return 0;
}

