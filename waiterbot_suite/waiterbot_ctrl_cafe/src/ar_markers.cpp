/*
 * ar_markers.cpp
 *
 *  Created on: Apr 6, 2013
 *      Author: jorge
 */

#include <dynamic_reconfigure/Reconfigure.h>

#include <yocs_math_toolkit/common.hpp>
#include <yocs_math_toolkit/geometry.hpp>

#include "waiterbot_ctrl_cafe/ar_markers.hpp"

namespace waiterbot
{

const uint32_t ARMarkersCafe::MARKERS_COUNT = 32;

ARMarkersCafe::ARMarkersCafe()
{
  // Invalid id until we localize it globally
  docking_marker_.id = std::numeric_limits<uint32_t>::max();
}

ARMarkersCafe::~ARMarkersCafe()
{
}

bool ARMarkersCafe::init()
{
  ROS_INFO("ARMarkersCafe Init");
  ros::NodeHandle nh, pnh("~");

  // Parameters
  pnh.param("global_frame",  global_frame_,  std::string("map"));
  pnh.param("odom_frame",    odom_frame_,    std::string("odom"));
  pnh.param("base_frame",    base_frame_,    std::string("base_footprint"));
  pnh.param("ar_markers/tf_broadcast_freq",  tf_broadcast_freq_,  0.0);  // disabled by default
  pnh.param("ar_markers/global_pose_conf",   global_pose_conf_,   0.8);
  pnh.param("ar_markers/docking_base_conf",  docking_base_conf_,  0.5);
  pnh.param("ar_markers/max_valid_d_inc",    max_valid_d_inc_,    0.8);
  pnh.param("ar_markers/max_valid_h_inc",    max_valid_h_inc_,    4.0);
  pnh.param("ar_markers/max_tracking_time",  max_tracking_time_,  2.0);
  pnh.param("ar_markers/min_penalized_dist", min_penalized_dist_, 1.4);
  pnh.param("ar_markers/max_reliable_dist",  max_reliable_dist_,  2.8);
  pnh.param("ar_markers/min_penalized_head", min_penalized_head_, 1.0);
  pnh.param("ar_markers/max_reliable_head",  max_reliable_head_,  1.4);

  pnh.param("ar_markers/global_pose_distance_max",   global_pose_distance_max_,   1.5);
  if (nh.getParam("ar_track_alvar/max_frequency", ar_tracker_freq_) == false)
  {
    ar_tracker_freq_ = 10.0;
    ROS_WARN("Cannot get AR tracker frequency; using default value (%f)", ar_tracker_freq_);
    ROS_WARN("Confidence evaluation can get compromised if this is not the right value!");
  }

  tracked_markers_sub_ = nh.subscribe("ar_track_alvar/ar_pose_marker", 1, &ARMarkersCafe::arMarkerCB, this);
  global_markers_sub_  = nh.subscribe(             "marker_pose_list", 1, &ARMarkersCafe::globalMarkersCB, this);

  // There are 18 different markers
  tracked_markers_.resize(ARMarkerTrackingDefaultParams::MARKERS_COUNT);

  if (tf_broadcast_freq_ > 0.0)
  {
    boost::thread(&ARMarkersCafe::broadcastMarkersTF, this);
  }

  // Disable tracking until needed
//  tracker_enabled_ = true;
//  disableTracker();

  return true;
}

void ARMarkersCafe::broadcastMarkersTF()
{
  ros::Rate rate(tf_broadcast_freq_);

  // TODO semantic map is not broadcasting this already?  only docking base no
  while (ros::ok())
  {
    char child_frame[32];
    tf::StampedTransform tf;
    tf.stamp_ = ros::Time::now();

    for (unsigned int i = 0; i <global_markers_.markers.size(); i++)
    {
      sprintf(child_frame, "%s_%d", "global_marker", global_markers_.markers[i].id);
      mtk::pose2tf(global_markers_.markers[i].pose, tf);
      tf.child_frame_id_ = child_frame;
      tf.stamp_ = ros::Time::now();
      tf_brcaster_.sendTransform(tf);
    }


    for (unsigned int i = 0; i <global_markers_mirrors.markers.size(); i++)
    {
      sprintf(child_frame, "%s_%d", "MIRROR", i);
      mtk::pose2tf(global_markers_mirrors.markers[i].pose, tf);
      tf.child_frame_id_ = child_frame;
      tf.stamp_ = ros::Time::now();
      tf_brcaster_.sendTransform(tf);
    }


// TODO remove definitively or recover id I finally decide not to include docking base on global markers
// CHANGED  look below
    if (docking_marker_.id != std::numeric_limits<uint32_t>::max())
    {
      sprintf(child_frame, "docking_base_%d", docking_marker_.id);
      mtk::pose2tf(docking_marker_.pose, tf);
      tf.child_frame_id_ = child_frame;
      tf.stamp_ = ros::Time::now();
      tf_brcaster_.sendTransform(tf);
    }

    rate.sleep();
  }
}

void ARMarkersCafe::publish_transform(const std::string parent, const std::string child,const tf::Transform& t)
{
  tf::StampedTransform tt(t, ros::Time::now(), parent, child);
  tf_brcaster_.sendTransform(tt);
}

void ARMarkersCafe::arMarkerCB(const ar_track_alvar::AlvarMarkers::ConstPtr& msg)
{
  ARMarkerTracking::arPoseMarkersCB(msg);
}

void ARMarkersCafe::globalMarkersCB(const ar_track_alvar::AlvarMarkers::ConstPtr& msg)
{
  // Just take first message; ignore the rest, as global markers list is not dynamic
  if ((global_markers_.markers.size() == 0) && (msg->markers.size() > 0))
  {
    global_markers_ = *msg;
    ROS_INFO("%lu global marker pose(s) received", global_markers_.markers.size());
    for (unsigned int i = 0; i < global_markers_.markers.size(); i++)
    {


      // Compensate the vertical alignment of markers and put at ground level to adopt navistack goals format
      tf::Transform tf(tf::createQuaternionFromYaw(tf::getYaw(global_markers_.markers[i].pose.pose.orientation) - M_PI/2.0),
                       tf::Vector3(global_markers_.markers[i].pose.pose.position.x, global_markers_.markers[i].pose.pose.position.y, 0.0));
      tf::StampedTransform marker_gb(tf, ros::Time::now(), global_frame_, "KKKK");

      // Half turn and translate to put goal at some distance in front of the marker
      tf::Transform in_front(tf::createQuaternionFromYaw(M_PI),
                             tf::Vector3(1.0, 0.0, 0.0));
      marker_gb *= in_front;

      ar_track_alvar::AlvarMarker kk;
      mtk::tf2pose(marker_gb, kk.pose);
      global_markers_mirrors.markers.push_back(kk);


      /*
      //TODO cheat for debuging;  remove
            if (global_markers_.markers[i].id >= 4)
            {
              ROS_DEBUG("Docking marker %d: %s", global_markers_.markers[i].id, mtk::pose2str(global_markers_.markers[i].pose.pose));
              docking_marker_ = global_markers_.markers[i];
              global_markers_.markers.pop_back();
            }
            else

      */
      ROS_DEBUG("Marker %d: %s", global_markers_.markers[i].id, mtk::pose2str(global_markers_.markers[i].pose.pose));
    }
  }
}

void ARMarkersCafe::customCB(const ar_track_alvar::AlvarMarkers& spotted_markers, const std::vector<TrackedMarker> &tracked_markers)
{
  for(unsigned int i = 0; i < spotted_markers.markers.size(); i++)
  {
    if (spotted_markers.markers[i].id >= tracked_markers.size())                                        
    {
       // A recognition error from Alvar markers tracker                                      
       ROS_WARN("Discarding AR marker with unrecognized id (%d)", spotted_markers.markers[i].id);
       continue;
    }

    const TrackedMarker& marker = tracked_markers[spotted_markers.markers[i].id];

    // Docking spotted!!!
    if( spotted_markers.markers[i].id == docking_marker_.id)
    {
      // This is the docking base marker! call the registered callbacks if it's reliable enough
      if (marker.confidence <= docking_base_conf_)
      {
        ROS_WARN_THROTTLE(1, "Ignoring docking base marker at it is not reliable enough (%f < %f)",
                              marker.confidence, docking_base_conf_);
        continue;
      }

      boost::shared_ptr<geometry_msgs::PoseStamped> ps(new geometry_msgs::PoseStamped());
      *ps = spotted_markers.markers[i].pose;
      ps->header = spotted_markers.markers[i].header;  // bloody alvar tracker doesn't fill pose's header
      base_spotted_cb_(ps, spotted_markers.markers[i].id);
    }

    // Global Marker
    ar_track_alvar::AlvarMarker global_marker;
    if (included(spotted_markers.markers[i].id, global_markers_, &global_marker) == true)
    {

      ROS_INFO("Distance3d    = %.2f",marker.distance);
      ROS_INFO("Distance2d    = %.2f",marker.distance2d);
      ROS_INFO("Heading       = %.2f",marker.heading);
      ROS_INFO("conf_distance = %.2f", marker.conf_distance);
      ROS_INFO("conf_heading  = %.2f",marker.conf_heading);

      ROS_INFO("Global Marker confidence = %.2f, pose_conf = %.2f",marker.confidence, global_pose_conf_);

      // This is a global marker! infer robot's global pose and call registered callbacks if it's reliable enough
      if (marker.confidence <= global_pose_conf_)
      {
        //if (msg->markers[i].id == 1)
        ROS_DEBUG_THROTTLE(1, "Discarding global marker at it is not reliable enough (%f < %f)",
                               marker.confidence, global_pose_conf_);
        continue;
      }

      boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped> pwcs(new geometry_msgs::PoseWithCovarianceStamped);

      // Marker tf on global reference system
      tf::StampedTransform marker_gb;
      mtk::pose2tf(global_marker.pose, marker_gb);

      // Marker tf on robot base reference system
      tf::StampedTransform marker_bs;
      if (getMarkerTf(base_frame_, "ar_marker", spotted_markers.markers[i].id, spotted_markers.markers[i].header.stamp, marker_bs, 0.4) == false)
      {
        // This should not happen unless AR tracker made a mistake, as we are using the timestamp he reported
        ROS_ERROR("Global marker spotted but we failed to get spotted marker -> base_footprint tf");
        continue;
      }

      // Calculate robot tf on global reference system multiplying the global marker tf (known a priori)
      // by marker tf on base reference system, that is, "subtract" the relative tf to the absolute one
      tf::Transform robot_gb = marker_gb * marker_bs.inverse();
      publish_transform("map", "robot",robot_gb);
      mtk::tf2pose(robot_gb, pwcs->pose.pose);

      pwcs->header.stamp = spotted_markers.markers[i].header.stamp;
      pwcs->header.frame_id = global_frame_;
      pwcs->pose.covariance[6*0+0] = 0.5 * 0.5;
      pwcs->pose.covariance[6*1+1] = 0.5 * 0.5;
      pwcs->pose.covariance[6*5+5] = M_PI/12.0 * M_PI/12.0;

      //if (msg->markers[i].id == 1)
      robot_pose_cb_(pwcs);

    }
  }
}

void ARMarkersCafe::print_stampedtransform(const std::string& name, tf::StampedTransform& t)
{
  tf::Vector3 p = t.getOrigin();
  tf::Quaternion q = t.getRotation();
  ROS_INFO("%s : %.2f %.2f %.2f,  %.2f %.2f %.2f %.2f ", name.c_str(), p.getX(),p.getY(), p.getZ(), q.getX(), q.getY(), q.getZ(), q.getW());
}

void ARMarkersCafe::print_transform(const std::string& name, tf::Transform& t)
{
  tf::Vector3 p = t.getOrigin();
  tf::Quaternion q = t.getRotation();
  ROS_INFO("%s : %.2f %.2f %.2f,  %.2f %.2f %.2f %.2f ", name.c_str(), p.getX(),p.getY(), p.getZ(), q.getX(), q.getY(), q.getZ(), q.getW());
}

bool ARMarkersCafe::spotDockMarker(uint32_t base_marker_id)
{
  for (unsigned int i = 0; i < spotted_markers_.markers.size(); i++)
  {
    if (spotted_markers_.markers[i].id == base_marker_id)
    {
      if (tracked_markers_[spotted_markers_.markers[i].id].confidence < 0.3)
      {
        ROS_WARN("Low confidence on spotted docking marker. Very dangerous...",
                 tracked_markers_[spotted_markers_.markers[i].id].confidence);
      }

      docking_marker_ = spotted_markers_.markers[i];
      docking_marker_.header.frame_id = global_frame_;
      docking_marker_.pose.header.frame_id = global_frame_;

      // Get marker tf on global reference system
      tf::StampedTransform marker_gb;
      if (getMarkerTf(global_frame_, "ar_marker", base_marker_id, docking_marker_.pose.header.stamp, marker_gb, 0.5) == false)
      {
        // This should not happen unless AR tracker made a mistake, as we are using the timestamp he reported
        ROS_ERROR("Docking base marker spotted but we failed to get its tf");
        return false;
      }

      // From now we consider it as another global marker
      mtk::tf2pose(marker_gb, docking_marker_.pose.pose);
      ROS_DEBUG("Docking AR marker registered with global pose: %.2f, %.2f, %.2f",
                docking_marker_.pose.pose.position.x, docking_marker_.pose.pose.position.y,
                tf::getYaw(docking_marker_.pose.pose.orientation));

//      CHANGED... doing this avoids reconfiguring in case of failure, and it's not a real advantage
//      global_markers_.markers.push_back(docking_marker_);
      return true;
    }
  }

  // Cannot spot docking marker
  return false;
}

bool ARMarkersCafe::getMarkerTf(const std::string& ref_frame, const std::string& prefix, uint32_t marker_id,
                               const ros::Time& timestamp, tf::StampedTransform& tf, const float timeout)
{
  char marker_frame[32];
  sprintf(marker_frame, "%s_%d", prefix.c_str(), marker_id);
  std::string target_frame(marker_frame);

  return getTf(ref_frame, target_frame, timestamp, tf, timeout);
}

bool ARMarkersCafe::getTf(const std::string& ref_frame, const std::string& marker_frame, 
                               const ros::Time& timestamp, tf::StampedTransform& tf, const float timeout)
{
  try
  {
    // Get marker tf on given reference system
    tf_listener_.waitForTransform(ref_frame, marker_frame, timestamp, ros::Duration(timeout));
    tf_listener_.lookupTransform(ref_frame, marker_frame, timestamp, tf);

    if (mtk::roll(tf) < -1.0)
    {
      // Sometimes markers are spotted "inverted" (pointing to -y); as we assume that all the markers are
      // aligned with y pointing up, x pointing right and z pointing to the observer, that's a recognition
      // error. We fix this flipping the tf.
      // TODO: while not to make tfs roll-invariant, so we avoid errors due to wrong markers alignment?
      tf::Transform flip(tf::createQuaternionFromRPY(0.0, 0.0, M_PI));
      tf *= flip;
    }
  }
  catch (tf::TransformException& e)
  {
    ROS_ERROR("Cannot get tf %s -> %s: %s", ref_frame.c_str(), marker_frame.c_str(), e.what());
    return false;
  }
  return true;
}


bool ARMarkersCafe::enableTracker()
{
  // TODO do not use by now because:
  //  - most times takes really long; I cannot find a pattern on timings
  //  - with -no kinect version of the tracker is not necessary anymore
  //  - a couple of times crashed and arduino gateway cpu usage spiked to 90%  NO IDEA WHY!!!!
  //  * Update:  retest with service call;  probably solves the last issue
  // TODO should I call waitForServer? here? on init? mollaio...
  return true;

//  if (tracker_enabled_ == true)
//    return true;
//
//  ros::Time t0 = ros::Time::now();
//  dynamic_reconfigure::Reconfigure srv;
//  srv.request.config.bools.resize(1);
//  srv.request.config.bools[0].name = "enabled";
//  srv.request.config.bools[0].value = true;
//
//  if (tracker_params_srv_.call(srv))
//  {
//    ROS_INFO("AR markers tracker enabled (%f seconds)", (ros::Time::now() - t0).toSec());
//    tracker_enabled_ = true;
//    return true;
//  }
//  else
//  {
//    ROS_ERROR("Failed to enable AR markers tracker (%f seconds)", (ros::Time::now() - t0).toSec());
//    return false;
//  }
}

bool ARMarkersCafe::disableTracker()
{
  // TODO do not use by now because:
  //  - disable takes really long (if I use again, use instead max_frequency: 1.0 (the minimum))
  //  - with -no kinect version of the tracker is not necessary anymore
  //  - a couple of times crashed and arduino gateway cpu usage spiked to 90%  NO IDEA WHY!!!!
  //  * Update:  retest with service call;  probably solves the last issue
  // TODO should I call waitForServer? here? on init? mollaio...
  return true;

//  if (tracker_enabled_ == false)
//    return true;
//
//  ros::Time t0 = ros::Time::now();
//  dynamic_reconfigure::Reconfigure srv;
//  srv.request.config.bools.resize(1);
//  srv.request.config.bools[0].name = "enabled";
//  srv.request.config.bools[0].value = false;
//
//  if (tracker_params_srv_.call(srv))
//  {
//    ROS_INFO("AR markers tracker disabled (%f seconds)", (ros::Time::now() - t0).toSec());
//    tracker_enabled_ = false;
//    return true;
//  }
//  else
//  {
//    ROS_ERROR("Failed to disable AR markers tracker (%f seconds)", (ros::Time::now() - t0).toSec());
//    return false;
//  }
}


void ARMarkersCafe::setRobotPoseCB(boost::function<void (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&)> cb)
{
  robot_pose_cb_ = cb;
}

void ARMarkersCafe::baseSpottedCB(boost::function<void (const geometry_msgs::PoseStamped::ConstPtr&, uint32_t)> cb)   
{
  base_spotted_cb_ = cb;
}

bool ARMarkersCafe::dockingBaseSpotted()
{
  return (docking_marker_.id != std::numeric_limits<uint32_t>::max()); 
}

const geometry_msgs::PoseStamped& ARMarkersCafe::getDockingBasePose() 
{ 
  return docking_marker_.pose; 
}


bool ARMarkersCafe::spotted(double younger_than, double min_confidence, ar_track_alvar::AlvarMarkers& spotted_markers)
{
  return ARMarkerTracking::spotted(younger_than, min_confidence, spotted_markers);
}
bool ARMarkersCafe::closest(double younger_than, double min_confidence, ar_track_alvar::AlvarMarker& closest_marker)
{
  return ARMarkerTracking::closest(younger_than, min_confidence, closest_marker);
}
bool ARMarkersCafe::spotted(double younger_than, const ar_track_alvar::AlvarMarkers& including, const ar_track_alvar::AlvarMarkers& excluding, ar_track_alvar::AlvarMarkers& spotted_markers)
{
  return ARMarkerTracking::spotted(younger_than, including, excluding, spotted_markers);
}
bool ARMarkersCafe::closest(const ar_track_alvar::AlvarMarkers& including, const ar_track_alvar::AlvarMarkers& excluding, ar_track_alvar::AlvarMarker& closest_marker)
{
  return ARMarkerTracking::closest(including, excluding, closest_marker);
}

bool ARMarkersCafe::spotted(double younger_than, double min_confidence, bool exclude_globals, ar_track_alvar::AlvarMarkers& spotted)
{
  if (spotted_markers_.markers.size() == 0)
    return false;

  if ((ros::Time::now() - spotted_markers_.markers[0].header.stamp).toSec() >= younger_than)
  {
    // We must check the timestamp from an element in the markers list, as the one on message's header is always zero!
    // WARNING: parameter younger_than must be high enough, as ar_track_alvar publish at Kinect rate but only updates
    // timestamps about every 0.1 seconds (and now we can set it to run slower, as frequency is a dynamic parameter!)
    ROS_WARN("Spotted markers too old:   %f  >=  %f",   (ros::Time::now() - spotted_markers_.markers[0].header.stamp).toSec(), younger_than);
    return false;
  }

  spotted.header = spotted_markers_.header;
  spotted.markers.clear();
  for (unsigned int i = 0; i < spotted_markers_.markers.size(); i++)
  {
    if ((exclude_globals == true) && (included(spotted_markers_.markers[i].id, global_markers_) == true))
      continue;

    if (tracked_markers_[spotted_markers_.markers[i].id].confidence >= min_confidence)
    {
      spotted.markers.push_back(spotted_markers_.markers[i]);
    }
  }

  return (spotted.markers.size() > 0);
}

bool ARMarkersCafe::closest(double younger_than, double min_confidence, bool exclude_globals, ar_track_alvar::AlvarMarker& closest)
{
  ar_track_alvar::AlvarMarkers spotted_markers;
  if (spotted(younger_than, min_confidence, exclude_globals, spotted_markers) == false)
    return false;

  double closest_dist = std::numeric_limits<double>::max();
  for (unsigned int i = 0; i < spotted_markers.markers.size(); i++)
  {
    double d = mtk::distance2D(spotted_markers.markers[i].pose.pose.position);
    if (d < closest_dist)
    {
      closest_dist = d;
      closest = spotted_markers.markers[i];
    }
  }

  return (closest_dist < std::numeric_limits<double>::max());
}

} /* namespace waiterbot */
