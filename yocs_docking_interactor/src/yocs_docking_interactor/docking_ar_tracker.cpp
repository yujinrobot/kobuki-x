/*  
 *  docking_ar_tracker.cpp
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/yujin_ocs/license/LICENSE
 */

#include "yocs_docking_interactor/ar_tracker.hpp"

namespace yocs_docking_interactor {

DockingARTracker::DockingARTracker(ros::NodeHandle& n): nh_(n) {
  init();
}

DockingARTracker::~DockingARTracker() {
}

bool DockingARTracker::init()
{
  ros::NodeHandle pnh("~");
  pnh.param("docking_ar_min_confidence", min_confidence_, 0.3);

  dock_marker_registered_ = false;

  // ar track alvar tracker handler
  srv_tracker_params_ = nh_.serviceClient<dynamic_reconfigure::Reconfigure>(DockingARTrackerDefaultParam::AR_TRACKER_SET_PARAM);
  tracker_enabled_ = false;

  global_marker_received_ = false;
  sub_global_markers_ = nh_.subscribe(DockingARTrackerDefaultParam::SUB_GLOBAL_MARKERS, 1, &DockingARTracker::processGlobalMarkers, this);
}

bool DockingARTracker::isReady()
{
  return global_marker_received_;
}

bool DockingARTracker::isDockRegistered()
{
  return dock_marker_registered_;
}

bool DockingARTracker::reset()
{
  dock_marker_registered_ = false;
  return true;
}

void DockingARTracker::processGlobalMarkers(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{

  for(unsigned int i=0; i < msg->markers.size(); i++)
  {
    ar_track_alvar_msgs::AlvarMarker m_right, m_left;

    m_right = msg->markers[i];
    m_left = msg->markers[i];
    m_left.id = m_left.id -3;

    global_markers_.markers.push_back(m_right);
    global_markers_.markers.push_back(m_left);
  }

  global_marker_received_ = true;
}

void DockingARTracker::customCB(const ar_track_alvar_msgs::AlvarMarkers& spotted_markers, const std::vector<yocs::TrackedMarker> &tracked_markers)
{
}

bool DockingARTracker::setClosestAsDockingMarker(int& id)
{
  bool success;
  // the closest ar marker is the docking marker.
  success = ARMarkerTracking::closest(1.0, min_confidence_, global_markers_,  docking_marker_in_robot_frame_);
  id = docking_marker_in_robot_frame_.id;
  return success;
}

bool DockingARTracker::registerDockingOnGlobalFrame(const std::string global_frame, const std::string base_frame,  std::string& message)
{
  ar_track_alvar_msgs::AlvarMarkers spotted_markers;
  ar_track_alvar_msgs::AlvarMarker current_dock_marker;

  if(spotted(1.0, min_confidence_, global_markers_, spotted_markers) == false)
  {
    message = "failed to spot dock marker";
    return false;
  }

  if(included(docking_marker_in_robot_frame_.id, spotted_markers, &current_dock_marker) == false)
  {
    message = "failed to spot dock marker";
    return false;
  }

  try
  {
    ROS_INFO_STREAM("" << current_dock_marker);
    geometry_msgs::PoseStamped after;
    docking_marker_in_robot_frame_ = current_dock_marker;
    docking_marker_in_robot_frame_.pose.header = docking_marker_in_robot_frame_.header;
    docking_marker_in_robot_frame_.pose.header.stamp = ros::Time::now();
    docking_marker_in_global_frame_ = docking_marker_in_robot_frame_;
    ROS_INFO_STREAM("" << docking_marker_in_robot_frame_);

    ROS_INFO("Get dock pose in global");
    getDockPoseInGlobal(global_frame, base_frame, docking_marker_in_robot_frame_.pose, after);

    ROS_INFO("Got get Dock pose in global frame");
    docking_marker_in_global_frame_.header = after.header;
    docking_marker_in_global_frame_.pose = after;
    getRobotPose(global_frame, base_frame, robot_dock_pose_);

    ROS_INFO("Got robot pose");
  }catch(tf::TransformException& e)
  {
//    ROS_ERROR("Cannot get tf %s -> %s : %s", docking_marker_in_robot_frame_.pose.header.frame_id.c_str(), global_frame.c_str(), e.what());
    std::stringstream ss;
    ss << "cannot get tranform to global frame. " << "Global Frame[" <<global_frame << "] base frame["<<base_frame <<"] " << e.what();
    message = ss.str();
    return false;
  }

  dock_marker_registered_ = true;
  message = "success";
  return true;
}

void DockingARTracker::getDockPoseInGlobal(const std::string& global_frame, const std::string& base_frame, const geometry_msgs::PoseStamped dock, geometry_msgs::PoseStamped& pose)
{

  // somehow it is better to wait 1 or 2sec to get the latest base pose in map frame. Without waiting, it gets old transform which is no longer valid.
  ros::Duration(2.0).sleep();

  // get global frame to base transform 
  tf::StampedTransform tf_global_frame_to_base_frame;
  tf::Transformer transformer;
  tf_listener_.waitForTransform(global_frame, base_frame, ros::Time(0), ros::Duration(1.0));
  tf_listener_.lookupTransform(global_frame, base_frame, ros::Time(0), tf_global_frame_to_base_frame);
  transformer.setTransform(tf_global_frame_to_base_frame);

  // get base to docking marker transform
  tf::StampedTransform tf_marker_to_base;
  geometry_msgs::PoseStamped dock_marker_in_base;
  ROS_INFO("Transform camera to base");
  tf_listener_.transformPose(base_frame, dock, dock_marker_in_base);
  mtk::pose2tf(dock_marker_in_base, tf_marker_to_base);
  tf_marker_to_base.child_frame_id_ = "dock";
  tf_marker_to_base.stamp_ = ros::Time::now();
  transformer.setTransform(tf_marker_to_base);

  // get transform from map to docking marker
  tf::StampedTransform tf_dock_to_global;
  transformer.lookupTransform(base_frame,  "dock", ros::Time(0), tf_dock_to_global);
    
  mtk::tf2pose(tf_dock_to_global, pose);
}

void DockingARTracker::getRobotPose(const std::string& global_frame, const std::string& base_frame, geometry_msgs::PoseStamped& pose) {
  geometry_msgs::PoseStamped robot_pose;
  tf::StampedTransform robot_tf;
  tf_listener_.lookupTransform(global_frame, base_frame, ros::Time(0.0), robot_tf);
  mtk::tf2pose(robot_tf, robot_pose); 

  pose = robot_pose;
}

void DockingARTracker::getRobotDockPose(geometry_msgs::PoseStamped& pose)
{
  pose = robot_dock_pose_;
}

bool DockingARTracker::isDockMarkerSpotted(geometry_msgs::PoseStamped& dock_pose, std::string& message)
{
  ar_track_alvar_msgs::AlvarMarkers spotted_markers;
  ar_track_alvar_msgs::AlvarMarker current_dock_marker;

  if(spotted(1.0, min_confidence_, global_markers_, spotted_markers) == false)
  {
    message = "failed to spot dock marker";
    return false;
  }

  if(included(docking_marker_in_robot_frame_.id, spotted_markers, &current_dock_marker) == false)
  {
    message = "failed to spot dock marker";
    return false;
  }
}
}
