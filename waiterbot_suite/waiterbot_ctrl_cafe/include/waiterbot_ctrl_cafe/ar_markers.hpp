/*
 * ar_markers.hpp
 *
 *  Created on: Apr 6, 2013
 *      Author: jorge
 */

#ifndef AR_MARKERS_HPP_
#define AR_MARKERS_HPP_

#include <boost/function.hpp>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ar_track_alvar/AlvarMarkers.h>
#include <waiterbot_ar_marker_tracking/tracking.hpp>

namespace waiterbot
{

class ARMarkersCafe : public ARMarkerTracking
{
public:
  static const uint32_t MARKERS_COUNT;

  ARMarkersCafe();
  virtual ~ARMarkersCafe();

  bool init();


  void arMarkerCB(const ar_track_alvar::AlvarMarkers::ConstPtr& msg);
  void setRobotPoseCB(boost::function<void (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&)> cb);
  void baseSpottedCB(boost::function<void (const geometry_msgs::PoseStamped::ConstPtr&, uint32_t)> cb);

  bool dockingBaseSpotted();
  const geometry_msgs::PoseStamped& getDockingBasePose();

  bool spotDockMarker(uint32_t base_marker_id);

  static bool enableTracker();
  static bool disableTracker();

  bool spotted(double younger_than, double min_confidence, ar_track_alvar::AlvarMarkers& spotted_markers);
  bool closest(double younger_than, double min_confidence, ar_track_alvar::AlvarMarker& closest_marker);
  bool spotted(double younger_than, const ar_track_alvar::AlvarMarkers& including, const ar_track_alvar::AlvarMarkers& excluding, ar_track_alvar::AlvarMarkers& spotted_markers);
  bool closest(const ar_track_alvar::AlvarMarkers& including, const ar_track_alvar::AlvarMarkers& excluding, ar_track_alvar::AlvarMarker& closest_marker);


  bool spotted(double younger_than, double min_confidence, bool exclude_globals, ar_track_alvar::AlvarMarkers& spotted_markers);
  bool closest(double younger_than, double min_confidence, bool exclude_globals, ar_track_alvar::AlvarMarker& closest_marker);




protected:
  void customCB(const ar_track_alvar::AlvarMarkers& spotted_markers, const std::vector<TrackedMarker> &tracked_markers);

private:

  // Confidence evaluation attributes

  double global_pose_conf_;
  double docking_base_conf_;

  // Other attributes
  std::string global_frame_;
  std::string odom_frame_;
  std::string base_frame_;

  tf::TransformListener    tf_listener_;
  tf::TransformBroadcaster tf_brcaster_;
  double                   tf_broadcast_freq_;  /**< Allows enabling tf broadcasting; mostly for debug */

  ar_track_alvar::AlvarMarker  docking_marker_;  /**< AR markers described in the semantic map */
  ar_track_alvar::AlvarMarkers global_markers_,global_markers_mirrors;  /**< AR markers described in the semantic map */

  bool               tracker_enabled_;
  ros::Subscriber    tracked_markers_sub_;
  ros::Subscriber    global_markers_sub_;

  // Registered callbacks to notify our observations

  boost::function<void (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&)>  robot_pose_cb_;
  boost::function<void (const geometry_msgs::PoseStamped::ConstPtr&, uint32_t)>    base_spotted_cb_;

  // Private methods
  void broadcastMarkersTF();
  void globalMarkersCB(const ar_track_alvar::AlvarMarkers::ConstPtr& msg);
  void arPoseMarkersCB(const ar_track_alvar::AlvarMarkers::ConstPtr& msg);

  bool getMarkerTf(const std::string& ref_frame, const std::string& prefix, uint32_t marker_id,
                     const ros::Time& timestamp, tf::StampedTransform& tf);

};

} /* namespace waiterbot */

#endif /* AR_MARKERS_HPP_ */
