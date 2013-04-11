/*
 * navigator.cpp
 *
 *  Created on: Apr 11, 2013
 *      Author: jorge
 */

#include <actionlib_msgs/GoalID.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/client/simple_action_client.h>

#include "waiterbot/common.hpp"
#include "waiterbot/navigator.hpp"

namespace waiterbot
{

Navigator::Navigator()
{
  // TODO Auto-generated constructor stub

}

Navigator::~Navigator()
{
  // TODO Auto-generated destructor stub
}

bool Navigator::init()
{
  ros::NodeHandle nh, pnh("~");

  pnh.param("autodocking_distance", autodocking_distance_, 0.5);

  issue_goal_pub_  = nh.advertise <geometry_msgs::PoseStamped> ("issue_goal",  1);
  cancel_goal_pub_ = nh.advertise <actionlib_msgs::GoalID>     ("cancel_goal", 1);

  return true;
}

void Navigator::dockInBase(geometry_msgs::PoseStamped base_marker)
{
  // Project a pose in front of the docking base and heading to it

  tf::StampedTransform marker_gb;   // docking base marker on global reference system
  pose2tf(base_marker, marker_gb);

  tf::Transform goal_gb(marker_gb); // global pose facing the base at
  tf::Transform r(tf::createQuaternionFromRPY(-M_PI/2.0, 0.0, M_PI/2.0));
  tf::Transform t(tf::Quaternion::getIdentity(), tf::Vector3(-autodocking_distance_, 0.0, -marker_gb.getOrigin().z));

  marker_gb *= r;  // rotate to adopt mobile base Euler angles, facing the marker
  marker_gb *= t;  // translate to put on the ground at some distance of the marker


}

} /* namespace waiterbot */
