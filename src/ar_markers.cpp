/*
 * ar_markers.cpp
 *
 *  Created on: Apr 6, 2013
 *      Author: jorge
 */

#include "waiterbot/ar_markers.hpp"

namespace waiterbot
{

ARMarkers::ARMarkers()
{
  global_markers_.markers.push_back(ar_track_alvar::AlvarMarker());  // TODO do from semantic map!!!!!!!!!!!!!!
  global_markers_.markers[0].pose.pose.position.x = 3.0;
  global_markers_.markers[0].pose.pose.position.y = 4.0;
  global_markers_.markers[0].pose.pose.position.x = 0.3;


}

ARMarkers::~ARMarkers()
{
  // TODO Auto-generated destructor stub
}

bool ARMarkers::init()
{
  ros::NodeHandle nh, pnh("~");

  // Parameters
  pnh.param("global_frame", global_frame_, std::string("map"));
  pnh.param("odom_frame",   odom_frame_,   std::string("odom"));
  pnh.param("base_frame",   base_frame_,   std::string("base_footprint"));

  ar_pose_sub_ = nh.subscribe("ar_pose_marker", 5, &ARMarkers::arPoseMarkerCB, this);

  // There are 18 different markers
  times_spotted_.resize(18, 0);

  return true;
}

void ARMarkers::arPoseMarkerCB(const ar_track_alvar::AlvarMarkers::Ptr& msg)
{
  // TODO MAke pointer!!!!  to avoid copying    but take care of multi-threading
  // more TODO:  inc confidence is very shitty as quality measure ->  we need a filter!!!  >>>   and also incorporate on covariance!!!!
  // and one more TODO:  sometimes markers are spotted "inverted" (pointing to -y) -> heuristic that says that y is always pointing up
  for (unsigned int i = 0; i < msg->markers.size(); i++)
  {
    if (msg->markers[i].id >= times_spotted_.size())
    {
      // A recognition error from Alvar markers tracker
      ROS_WARN("Discarding AR marker with unrecognized id (%d)", msg->markers[i].id);
      continue;
    }

    times_spotted_[msg->markers[i].id] += 2;

    ar_track_alvar::AlvarMarker global_marker;
    if (included(msg->markers[i].id, global_markers_, &global_marker) == true)
    {
      boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped> pwcs(new geometry_msgs::PoseWithCovarianceStamped);

      std::string marker_frame("ar_marker_");
      marker_frame += msg->markers[i].id;

      tf::StampedTransform marker_gb; // marker on global reference system
      tf_listener_.waitForTransform(global_frame_, marker_frame, ros::Time(0), ros::Duration(0.05));
      tf_listener_.lookupTransform(global_frame_, marker_frame, ros::Time(0), marker_gb);

      tf::StampedTransform robot_mk;  // robot on marker reference system
      tf_listener_.waitForTransform(marker_frame, base_frame_, ros::Time(0), ros::Duration(0.05));
      tf_listener_.lookupTransform(marker_frame, base_frame_, ros::Time(0), robot_mk);

      tf::Transform robot_gb = marker_gb*robot_mk;

      tf2pose(robot_gb, pwcs->pose.pose);
      pwcs->header.stamp = msg->header.stamp;
      pwcs->header.frame_id = global_frame_;

      robot_pose_cb_(pwcs);
    }

    ROS_ERROR("MK  %d  %d", spotted_markers_.markers[i].id, times_spotted_[spotted_markers_.markers[i].id]);
  }
  spotted_markers_ = *msg;

  for (unsigned int i = 0; i < times_spotted_.size(); i++)
  {
    if (times_spotted_[i] > 0)
      times_spotted_[i]--;
  }
}

bool ARMarkers::spotted(double younger_than,
                        const ar_track_alvar::AlvarMarkers& including,
                        const ar_track_alvar::AlvarMarkers& excluding,
                              ar_track_alvar::AlvarMarkers& spotted)
{
  if ((ros::Time::now() - spotted_markers_.header.stamp).toSec() >= younger_than)
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
      double d = distance(spotted_markers_.markers[i].pose.pose.position, geometry_msgs::Point());
      if (d < closest_dist)
      {
        closest_dist = d;
        closest = spotted_markers_.markers[i];
      }
    }
  }

  return (closest_dist < std::numeric_limits<double>::max());
}

bool ARMarkers::spotted(double younger_than, int min_confidence, bool exclude_globals,
                        ar_track_alvar::AlvarMarkers& spotted)
{
  if ((ros::Time::now() - spotted_markers_.header.stamp).toSec() >= younger_than)
  {
    ROS_ERROR("OOOOOOOOOOOOOOOOOOLD");
    return false;
  }

  spotted.header = spotted_markers_.header;
  spotted.markers.clear();
  for (unsigned int i = 0; i < spotted_markers_.markers.size(); i++)
  {
    if ((exclude_globals == true) && (included(spotted_markers_.markers[i].id, global_markers_) == true))
      continue;

    if (times_spotted_[spotted_markers_.markers[i].id] >= min_confidence)
    {
      spotted.markers.push_back(spotted_markers_.markers[i]);
    }
  }

  return (spotted.markers.size() > 0);
}

bool ARMarkers::closest(double younger_than, int min_confidence, bool exclude_globals,
                        ar_track_alvar::AlvarMarker& closest)
{
  ar_track_alvar::AlvarMarkers spotted_markers;
  if (spotted(younger_than, min_confidence, exclude_globals, spotted_markers) == false)
    return false;

  double closest_dist = std::numeric_limits<double>::max();
  for (unsigned int i = 0; i < spotted_markers.markers.size(); i++)
  {
    double d = distance(spotted_markers.markers[i].pose.pose.position, geometry_msgs::Point());
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
      if (spotted_markers_.markers[i].confidence < 2)
      {
        ROS_WARN("Low confidence on spotted docking marker. Dangerous...", spotted_markers_.markers[i].confidence);
        // TODO   this can be catastrophic if we are very unlucky
      }

      base_marker_id_ = base_marker_id;
      docking_marker_ = spotted_markers_.markers[i];
      docking_marker_.header.frame_id = global_frame_;
      docking_marker_.pose.header.frame_id = global_frame_;

      std::string marker_frame("ar_marker_");
      marker_frame += base_marker_id;

      try
      {
        tf::StampedTransform marker_gb; // marker on global reference system
        tf_listener_.waitForTransform(global_frame_, marker_frame, ros::Time(0), ros::Duration(0.05));
        tf_listener_.lookupTransform(global_frame_, marker_frame, ros::Time(0), marker_gb);

//        tf::transformStampedTFToMsg(marker_gb.,
//        docking_marker_.pose.pose.position

//        tf::Transform grasp_tf(gripperRotation, gripperOrigin);
         tf::Stamped<tf::Pose> marker_tf_pose(marker_gb, ros::Time::now(), global_frame_);
    //     geometry_msgs::PoseStamped msg;
         tf::poseStampedTFToMsg(marker_tf_pose, docking_marker_.pose);

        global_markers_.markers.push_back(docking_marker_);
        ROS_DEBUG("Docking AR marker registered with global pose: %.2f, %.2f              , %s",
                  docking_marker_.pose.pose.position.x, docking_marker_.pose.pose.position.y,
                  docking_marker_.pose.header.frame_id.c_str());
        return true;
      }
      catch (tf::TransformException& e)
      {
        ROS_ERROR("Cannot get tf %s -> %s: %s", global_frame_.c_str(), marker_frame.c_str(), e.what());
        return false;
      }
    }
  }

  ROS_WARN("Cannot spot docking marker");
  return false;
}

void ARMarkers::tf2pose(const tf::Transform& tf, geometry_msgs::Pose& pose)
{
  pose.position.x = tf.getOrigin().x();
  pose.position.y = tf.getOrigin().y();
  pose.position.z = tf.getOrigin().z();
  tf::quaternionTFToMsg(tf.getRotation(), pose.orientation);
}


} /* namespace waiterbot */
