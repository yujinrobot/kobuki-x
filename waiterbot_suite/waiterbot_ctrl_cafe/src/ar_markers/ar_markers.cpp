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
  // configuration for ar pair tracking
  pnh.param("global_marker_prefix", global_marker_prefix_, std::string("global_marker"));
  pnh.param("global_marker_mirror_prefix", global_marker_mirror_prefix_, std::string("MIRROR"));
  pnh.param("target_frame_postfix", target_frame_postfix_, std::string("target"));
  pnh.param("ar_pair_baseline", baseline_, 0.28);
  pnh.param("ar_pair_target_offset", target_offset_, 0.5);

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

  update_ar_pairs_pub_ = nh.advertise<yocs_msgs::ARPairList>("update_ar_pairs", 1, true);

  // There are 18 different markers
  tracked_markers_.resize(yocs::ARMarkerTrackingDefaultParams::MARKERS_COUNT);

  if (tf_broadcast_freq_ > 0.0)
  {
    boost::thread(&ARMarkersCafe::broadcastMarkersTF, this);
  }

  return true;
}

void ARMarkersCafe::arMarkerCB(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
  ARMarkerTracking::arPoseMarkersCB(msg);
}

void ARMarkersCafe::globalMarkersCB(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
  // Just take first message; ignore the rest, as global markers list is not dynamic
  global_markers_ = *msg;

  createFixedMarkers();
  createMirrorMarkers();
  ROS_INFO("%lu global marker pose(s) received", global_markers_.markers.size());

  notifyARPairTracker();
}

void ARMarkersCafe::customCB(const ar_track_alvar_msgs::AlvarMarkers& spotted_markers, const std::vector<yocs::TrackedMarker> &tracked_markers)
{
  for(unsigned int i = 0; i < spotted_markers.markers.size(); i++)
  {
    if (spotted_markers.markers[i].id >= tracked_markers.size())                                        
    {
       // A recognition error from Alvar markers tracker                                      
       ROS_WARN("Discarding AR marker with unrecognized id (%d)", spotted_markers.markers[i].id);
       continue;
    }

    const yocs::TrackedMarker& marker = tracked_markers[spotted_markers.markers[i].id];

    // Docking spotted!!!
    if( spotted_markers.markers[i].id == docking_marker_.id)
    {
      // This is the docking base marker! call the registered callbacks if it's reliable enough
      if (marker.conf_distance  * marker.stability * marker.persistence <= docking_base_conf_)
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
  }
}

void ARMarkersCafe::notifyARPairTracker()
{
  unsigned int i;
  yocs_msgs::ARPairList l;

  // at the moment, it is hard to record both left and right marker id.
  // Instead, it use the given global marker id as left id and leftid -3 as right id.
  for(i = 0; i < global_markers_.markers.size(); i++)
  {
    ar_track_alvar_msgs::AlvarMarker m = global_markers_.markers[i];
    yocs_msgs::ARPair p;
    char frame[32];
    sprintf(frame, "%s_%d_%s", global_marker_prefix_.c_str(), m.id, target_frame_postfix_.c_str());

    p.left_id = m.id;
    p.right_id = p.left_id - 3;
    p.baseline = baseline_;
    p.target_offset = target_offset_;
    p.target_frame = frame;
    l.pairs.push_back(p);
  }

  update_ar_pairs_pub_.publish(l);
}



bool ARMarkersCafe::spotDockMarker(uint32_t base_marker_id)
{
  for (unsigned int i = 0; i < spotted_markers_.markers.size(); i++)
  {
    if (spotted_markers_.markers[i].id == base_marker_id)
    {
      if (tracked_markers_[spotted_markers_.markers[i].id].confidence < 0.3)
      {
        ROS_WARN("Low confidence[%.2f] on spotted docking marker. Very dangerous...",
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

} /* namespace waiterbot */
