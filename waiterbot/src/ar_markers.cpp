/*
 * ar_markers.cpp
 *
 *  Created on: Apr 6, 2013
 *      Author: jorge
 */

#include <dynamic_reconfigure/Reconfigure.h>

#include <yocs_math_toolkit/common.hpp>
#include <yocs_math_toolkit/geometry.hpp>

#include "waiterbot/ar_markers.hpp"

namespace waiterbot
{

const uint32_t ARMarkers::MARKERS_COUNT = 32;

ARMarkers::ARMarkers()
{
  // Invalid id until we localize it globally
  docking_marker_.id = std::numeric_limits<uint32_t>::max();

/*
//TODO TEMPORAL FOR DEBUG

  docking_marker_.id = 6;
  docking_marker_.pose.header.frame_id = "map";
  docking_marker_.pose.pose.position.x = -0.1;
  docking_marker_.pose.pose.position.y = -1.9;
  docking_marker_.pose.pose.position.z = 0.1;
  docking_marker_.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI/2.0, 0.0, M_PI);
*/
}

ARMarkers::~ARMarkers()
{
}

bool ARMarkers::init()
{
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

  if (nh.getParam("ar_track_alvar/max_frequency", ar_tracker_freq_) == false)
  {
    ar_tracker_freq_ = 10.0;
    ROS_WARN("Cannot get AR tracker frequency; using default value (%f)", ar_tracker_freq_);
    ROS_WARN("Confidence evaluation can get compromised if this is not the right value!");
  }

  tracked_markers_sub_ = nh.subscribe("ar_track_alvar/ar_pose_marker", 1, &ARMarkers::arPoseMarkersCB, this);
  global_markers_sub_  = nh.subscribe(             "marker_pose_list", 1, &ARMarkers::globalMarkersCB, this);

  tracker_params_srv_  = nh.serviceClient<dynamic_reconfigure::Reconfigure>("ar_track_alvar/set_parameters");

  // There are 18 different markers
  tracked_markers_.resize(MARKERS_COUNT);

  if (tf_broadcast_freq_ > 0.0)
  {
    boost::thread(&ARMarkers::broadcastMarkersTF, this);
  }


  // Disable tracking until needed
//  tracker_enabled_ = true;
//  disableTracker();

  return true;
}

void ARMarkers::broadcastMarkersTF()
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
      sprintf(child_frame, "%s_%d", i != docking_marker_.id?"global_marker":"docking_base", global_markers_.markers[i].id);
      mtk::pose2tf(global_markers_.markers[i].pose, tf);
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

void ARMarkers::globalMarkersCB(const ar_track_alvar::AlvarMarkers::ConstPtr& msg)
{
  // Just take first message; ignore the rest, as global markers list is not dynamic
  if ((global_markers_.markers.size() == 0) && (msg->markers.size() > 0))
  {
    global_markers_ = *msg;
    ROS_INFO("%lu global marker pose(s) received", global_markers_.markers.size());
    for (unsigned int i = 0; i < global_markers_.markers.size(); i++)
    {
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

void ARMarkers::arPoseMarkersCB(const ar_track_alvar::AlvarMarkers::ConstPtr& msg)
{
  // TODO: use confidence to incorporate covariance to global poses

  // Make thresholds relative to the tracking frequency (as it can be dynamically changed)
  int obs_list_max_size  = (int)round(max_tracking_time_*ar_tracker_freq_);
  double max_valid_d_inc = max_valid_d_inc_/ar_tracker_freq_;
  double max_valid_h_inc = max_valid_h_inc_/ar_tracker_freq_;

  for (unsigned int i = 0; i < msg->markers.size(); i++)
  {
    if (msg->markers[i].id >= tracked_markers_.size())
    {
      // A recognition error from Alvar markers tracker
      ROS_WARN("Discarding AR marker with unrecognized id (%d)", msg->markers[i].id);
      continue;
    }

    // Confidence evaluation
    TrackedMarker& marker = tracked_markers_[msg->markers[i].id];
    marker.distance = mtk::distance3D(msg->markers[i].pose.pose);
    marker.heading  = tf::getYaw(msg->markers[i].pose.pose.orientation) + M_PI/2.0;
    // WARN: note that heading evaluation also assumes vertically aligned markers

//    tf::Transform tf;
//    tf::poseMsgToTF(msg->markers[i].pose.pose, tf);
//    double r, p, y;
//    tf.getBasis().getRPY(r, p, y);
//    ROS_DEBUG("%f     %f       %f  %f  %f", marker.heading, std::min(1.0, 1.0/std::pow(std::abs(marker.heading), 2)), r, p, y);

    int position = 0;
    ros::Time now = ros::Time::now();
    geometry_msgs::PoseStamped prev = msg->markers[i].pose;
    for (ObsList::iterator it = marker.obs_list_.begin(); it != marker.obs_list_.end(); ++it)
    {
      double age = (now - it->header.stamp).toSec();
      if (age > ((position + 1)/ar_tracker_freq_) + 1.0)
      {
        int s0 = marker.obs_list_.size();
        marker.obs_list_.erase(it, marker.obs_list_.end());
        int s1 = marker.obs_list_.size();
        ROS_DEBUG("%d observations discarded (fist one with position %d in the list) for being %f seconds old ( > %f)",
                  s0 - s1, position, age, ((position + 1)/ar_tracker_freq_) + 1.0);
        break;
      }

      if ((mtk::distance3D(prev.pose, it->pose) > max_valid_d_inc) ||
          (std::abs(mtk::minAngle(prev.pose, it->pose)) > max_valid_h_inc))
      {
        // Incoherent observation; stop going over the list and use current position value to fill confidence values
//        ROS_ERROR("%d  BREAK at %d   %f  %f     %f   %f        %f", msg->markers[i].id, position, mtk::distance3D(prev.pose, it->pose),  mtk::minAngle(prev.pose, it->pose), max_valid_d_inc, max_valid_h_inc, ar_tracker_freq_);
        break;
      }

      prev = *it;
      position++;
    }

    marker.conf_distance = marker.distance <= min_penalized_dist_ ? 1.0
                         : marker.distance >= max_reliable_dist_  ? 0.0
                         : 1.0 - std::pow((marker.distance - min_penalized_dist_) / (max_reliable_dist_ - min_penalized_dist_), 2);
    marker.conf_heading  = std::abs(marker.heading) <= min_penalized_head_ ? 1.0
                         : std::abs(marker.heading) >= max_reliable_head_  ? 0.0
                         : 1.0 - std::pow((std::abs(marker.heading) - min_penalized_head_) / (max_reliable_head_ - min_penalized_head_), 2);
    marker.stability     = marker.obs_list_.size() == 0 ? 0.0 : std::sqrt((double)position/(double)marker.obs_list_.size());
    marker.persistence   = std::sqrt((double)marker.obs_list_.size()/(double)obs_list_max_size);
    marker.confidence    = marker.conf_distance*marker.conf_heading*marker.stability*marker.persistence;
    marker.obs_list_.insert(marker.obs_list_.begin(), msg->markers[i].pose);
    marker.obs_list_.begin()->header = msg->markers[i].header;  // bloody alvar tracker doesn't fill pose's header
    if (marker.obs_list_.size() > obs_list_max_size)
      marker.obs_list_.pop_back();

//    ROS_DEBUG_STREAM(msg->markers[i].id << ":  "
//                 /* << marker.distance << "   " << marker.heading << "  " */<< marker.confidence << "      "
//                  << marker.conf_distance << "   " << marker.conf_heading << "   "
//                  << marker.stability << "   "<< marker.persistence << position << "   " << "   " << marker.obs_list_.size());

    if (msg->markers[i].id == docking_marker_.id)
    {
      // This is the docking base marker! call the registered callbacks if it's reliable enough
      if (marker.confidence <= docking_base_conf_)
      {
        ROS_WARN_THROTTLE(1, "Ignoring docking base marker at it is not reliable enough (%f < %f)",
                              marker.confidence, docking_base_conf_);
        continue;
      }

      boost::shared_ptr<geometry_msgs::PoseStamped> ps(new geometry_msgs::PoseStamped());
      *ps = msg->markers[i].pose;
      ps->header = msg->markers[i].header;  // bloody alvar tracker doesn't fill pose's header
      base_spotted_cb_(ps, msg->markers[i].id);
    }

    ar_track_alvar::AlvarMarker global_marker;
    if (included(msg->markers[i].id, global_markers_, &global_marker) == true)
    {
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
      if (getMarkerTf(base_frame_, msg->markers[i].id, msg->markers[i].header.stamp, marker_bs) == false)
      {
        // This should not happen unless AR tracker made a mistake, as we are using the timestamp he reported
        ROS_ERROR("Global marker spotted but we failed to get its tf");
        continue;
      }

      // Calculate robot tf on global reference system multiplying the global marker tf (known a priori)
      // by marker tf on base reference system, that is, "subtract" the relative tf to the absolute one
      tf::Transform robot_gb = marker_gb*marker_bs.inverse();
      mtk::tf2pose(robot_gb, pwcs->pose.pose);

      pwcs->header.stamp = msg->markers[i].header.stamp;
      pwcs->header.frame_id = global_frame_;

      //if (msg->markers[i].id == 1)
      robot_pose_cb_(pwcs);

      //< DEBUG
      // tf::StampedTransform tf1(marker_bs, ros::Time::now(),  base_frame_, "AR_MK");
      // tf::StampedTransform tf2(robot_gb, ros::Time::now(),  "map", "ROBOT");
      // tf_brcaster_.sendTransform(tf1);
      // tf_brcaster_.sendTransform(tf2);
      //>
    }
  }
  spotted_markers_ = *msg;
}

bool ARMarkers::spotted(double younger_than,
                           const ar_track_alvar::AlvarMarkers& including,
                           const ar_track_alvar::AlvarMarkers& excluding,
                                  ar_track_alvar::AlvarMarkers& spotted)
{
  if (spotted_markers_.markers.size() == 0)
    return false;

  if ((ros::Time::now() - spotted_markers_.markers[0].header.stamp).toSec() >= younger_than)
  {
    return false;
  }

  spotted.header = spotted_markers_.header;
  spotted.markers.clear();
  for (unsigned int i = 0; i < spotted_markers_.markers.size(); i++)
  {
    if ((included(spotted_markers_.markers[i].id, including) == true) &&
        (excluded(spotted_markers_.markers[i].id, excluding) == true))
    {
      spotted.markers.push_back(spotted_markers_.markers[i]);
    }
  }

  return (spotted.markers.size() > 0);
}

bool ARMarkers::closest(const ar_track_alvar::AlvarMarkers& including,
                           const ar_track_alvar::AlvarMarkers& excluding,
                                  ar_track_alvar::AlvarMarker& closest)
{
  double closest_dist = std::numeric_limits<double>::max();
  for (unsigned int i = 0; i < spotted_markers_.markers.size(); i++)
  {
    if ((included(spotted_markers_.markers[i].id, including) == true) &&
        (excluded(spotted_markers_.markers[i].id, excluding) == true))
    {
      double d = mtk::distance2D(spotted_markers_.markers[i].pose.pose.position);
      if (d < closest_dist)
      {
        closest_dist = d;
        closest = spotted_markers_.markers[i];
      }
    }
  }

  return (closest_dist < std::numeric_limits<double>::max());
}

bool ARMarkers::spotted(double younger_than, double min_confidence, bool exclude_globals,
                           ar_track_alvar::AlvarMarkers& spotted)
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

bool ARMarkers::closest(double younger_than, double min_confidence, bool exclude_globals,
                           ar_track_alvar::AlvarMarker& closest)
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

bool ARMarkers::spotDockMarker(uint32_t base_marker_id)
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
      if (getMarkerTf(global_frame_, base_marker_id, docking_marker_.pose.header.stamp, marker_gb) == false)
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

bool ARMarkers::getMarkerTf(const std::string& ref_frame, uint32_t marker_id,
                               const ros::Time& timestamp, tf::StampedTransform& tf)
{
  char marker_frame[32];
  sprintf(marker_frame, "ar_marker_%d", marker_id);

  try
  {
    // Get marker tf on given reference system
    tf_listener_.waitForTransform(ref_frame, marker_frame, timestamp, ros::Duration(0.05));
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
    ROS_WARN("Cannot get tf %s -> %s: %s", ref_frame.c_str(), marker_frame, e.what());
    return false;
  }
  return true;
}

bool ARMarkers::enableTracker()
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

bool ARMarkers::disableTracker()
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


bool ARMarkers::setTrackerFreq(double frequency)
{
//  return true;

  ros::Time t0 = ros::Time::now();
  dynamic_reconfigure::Reconfigure srv;
  srv.request.config.doubles.resize(1);
  srv.request.config.doubles[0].name = "max_frequency";
  srv.request.config.doubles[0].value = frequency;

  if (tracker_params_srv_.call(srv))
  {
    ROS_INFO("AR markers tracker frequency changed to %f (%f seconds)", frequency, (ros::Time::now() - t0).toSec());
    ar_tracker_freq_ = frequency;
    return true;
  }
  else
  {
    ROS_ERROR("Failed to change AR markers tracker frequency (%f seconds)", frequency, (ros::Time::now() - t0).toSec());
    return false;
  }
}


} /* namespace waiterbot */
