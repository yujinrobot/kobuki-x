/*
 * ar_marker_processor.hpp
 *
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/kobuki-x/license/LICENSE
 *
 *  Created on: Apr 6, 2013
 *      Author: jorge
 *  Modified on: Dec, 2013 
 *      Jihoon Lee
 */

#include "waiterbot_sensors/ar_marker_processor.hpp"

namespace waiterbot
{
  ARMarkerProcessor::ARMarkerProcessor() { 
    init(); 
  }

  ARMarkerProcessor::~ARMarkerProcessor() {}

  bool ARMarkerProcessor::init()
  {
    ros::NodeHandle nh, pnh("~");

    // Parameters
    pnh.param("global_frame",       global_frame_,           ARMarkerProcessorDefaultParams::GLOBAL_FRAME);
    pnh.param("odom_frame",         odom_frame_,             ARMarkerProcessorDefaultParams::ODOM_FRAME); 
    pnh.param("base_frame",         base_frame_,             ARMarkerProcessorDefaultParams::BASE_FRAME); 
    pnh.param("tf_broadcast_freq",  tf_broadcast_freq_,      ARMarkerProcessorDefaultParams::TF_BROADCAST_FREQ); // disabled by default
    pnh.param("global_pose_conf",   global_pose_conf_,       ARMarkerProcessorDefaultParams::GLOBAL_POSE_CONF);
    pnh.param("docking_base_conf",  docking_base_conf_,      ARMarkerProcessorDefaultParams::DOCKING_BASE_CONF);
    pnh.param("max_valid_d_inc",    max_valid_d_inc_,        ARMarkerProcessorDefaultParams::MAX_VALID_D_INC);
    pnh.param("max_valid_h_inc",    max_valid_h_inc_,        ARMarkerProcessorDefaultParams::MAX_VALID_H_INC);
    pnh.param("max_tracking_time",  max_tracking_time_,      ARMarkerProcessorDefaultParams::MAX_TRACKING_TIME);
    pnh.param("min_penalized_dist", min_penalized_dist_,     ARMarkerProcessorDefaultParams::MIN_PENALIZED_DIST);
    pnh.param("max_reliable_dist",  max_reliable_dist_,      ARMarkerProcessorDefaultParams::MAX_RELIABLE_DIST);
    pnh.param("min_penalized_head", min_penalized_head_,     ARMarkerProcessorDefaultParams::MIN_PENALIZED_HEAD);
    pnh.param("max_reliable_head",  max_reliable_head_,      ARMarkerProcessorDefaultParams::MAX_RELIABLE_HEAD);

    global_marker_localization_ = false;
    if(pnh.getParam("global_marker_filename",        global_marker_filename_) == false)
    {
      ROS_ERROR("global marker filename parameter is not set!. Cannot use ar_based localization");
    }

    if (nh.getParam("ar_track_alvar/max_frequency", ar_tracker_freq_) == false)
    {
      ar_tracker_freq_ = ARMarkerProcessorDefaultParams::AR_TRACKER_FREQ;
      ROS_WARN("Cannot get AR tracker frequency; using default value (%f)", ar_tracker_freq_);
      ROS_WARN("Confidence evaluation can get compromised if this is not the right value!");
    }

    sub_ar_markers_ = nh.subscribe("ar_track_alvar/ar_pose_marker", 1, &ARMarkerProcessor::arPoseMarkersCB, this);

    // There are 18 different markers
    tracked_markers_.resize(ARMarkerProcessorDefaultParams::MARKERS_COUNT);

    return true;
  }

  void ARMarkerProcessor::loadGlobalMarkers()
  {
    if(loadAlvarMarkersFromYaml(global_marker_filename_, global_markers_) == false)
    {
      ROS_WARN("Cannot use global marker based localization");
      global_marker_localization_ = false;
      return;
    }

    // Just take first message; ignore the rest, as global markers list is not dynamic
    ROS_INFO("%lu global marker pose(s) received", global_markers_.markers.size());
    for (unsigned int i = 0; i < global_markers_.markers.size(); i++)
    {
      // Compensate the vertical alignment of markers and put at ground level to adopt navistack goals format
      tf::Transform tf(tf::createQuaternionFromYaw(tf::getYaw(global_markers_.markers[i].pose.pose.orientation) - M_PI/2.0),
                       tf::Vector3(global_markers_.markers[i].pose.pose.position.x, global_markers_.markers[i].pose.pose.position.y, 0.0));
      tf::StampedTransform marker_gb(tf, ros::Time::now(), global_frame_, "global_marker_frame");

      // Half turn and translate to put goal at some distance in front of the marker
      tf::Transform in_front(tf::createQuaternionFromYaw(M_PI),
                             tf::Vector3(1.0, 0.0, 0.0));
      marker_gb *= in_front;

      ar_track_alvar::AlvarMarker kk;
      mtk::tf2pose(marker_gb, kk.pose);
      global_markers_mirrors_.markers.push_back(kk);

      ROS_DEBUG("Marker %d: %s", global_markers_.markers[i].id, mtk::pose2str(global_markers_.markers[i].pose.pose));
    }
  }

  void ARMarkerProcessor::arPoseMarkersCB(const ar_track_alvar::AlvarMarkers::ConstPtr& msg)
  {
    // TODO: use confidence to incorporate covariance to global poses

    // Make thresholds relative to the tracking frequency (as it can be dynamically changed)
    /*
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
  //                 << marker.distance << "   " << marker.heading << "  " << marker.confidence << "      "
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
    */
  }

  bool ARMarkerProcessor::spotted(double younger_than,
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

  bool ARMarkerProcessor::closest(const ar_track_alvar::AlvarMarkers& including, const ar_track_alvar::AlvarMarkers& excluding, ar_track_alvar::AlvarMarker& closest)
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

  bool ARMarkerProcessor::spotted(double younger_than, double min_confidence, bool exclude_globals, ar_track_alvar::AlvarMarkers& spotted)
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

  bool ARMarkerProcessor::closest(double younger_than, double min_confidence, bool exclude_globals, ar_track_alvar::AlvarMarker& closest)
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

  // check if the given id ar_marker is in the list. If yes, return the full ar marker data
  bool ARMarkerProcessor::included(const uint32_t id, const ar_track_alvar::AlvarMarkers& v, ar_track_alvar::AlvarMarker* e)
  {
    for (unsigned int i = 0; i < v.markers.size(); i++)
    {
      if (id == v.markers[i].id)
      {
        if (e != NULL)
          *e = v.markers[i];

        return true;
      }
    }

    return false;
  }

  // Check if the given id is in the list of ar markers
  bool ARMarkerProcessor::excluded(const uint32_t id, const ar_track_alvar::AlvarMarkers& v)
  {
    for (unsigned int i = 0; i < v.markers.size(); i++)
    {
      if (id == v.markers[i].id)
        return false;
    }

    return true;
  }

  void ARMarkerProcessor::spin()
  {
    loadGlobalMarkers();

    // Broadcasts the spotted ar markers tf

    if (tf_broadcast_freq_ > 0.0)
    {
      ros::Rate rate(tf_broadcast_freq_);

      while(ros::ok())
      {
        broadcastMarkersTF();
        rate.sleep();
      }
    }                                                                 
    else {
      ros::spin();
    }
  }

} // waiterbot namespace
