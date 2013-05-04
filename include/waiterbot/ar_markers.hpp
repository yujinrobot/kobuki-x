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

#include "waiterbot/common.hpp"

namespace waiterbot
{

class ARMarkers
{
public:
  ARMarkers();
  virtual ~ARMarkers();

  bool init();

  void setRobotPoseCB(boost::function<void (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&)> cb)
  {
    robot_pose_cb_ = cb;
  };

  void baseSpottedCB(boost::function<void (const geometry_msgs::PoseStamped::ConstPtr&, uint32_t)> cb)
  {
    base_spotted_cb_ = cb;
  };

  bool dockingBaseSpotted() { return  (docking_marker_.id != std::numeric_limits<uint32_t>::max()); }
  const geometry_msgs::PoseStamped& getDockingBasePose() { return docking_marker_.pose; };

  bool spotted(double younger_than,
               const ar_track_alvar::AlvarMarkers& including, const ar_track_alvar::AlvarMarkers& excluding,
               ar_track_alvar::AlvarMarkers& spotted_markers);
  bool closest(const ar_track_alvar::AlvarMarkers& including, const ar_track_alvar::AlvarMarkers& excluding,
               ar_track_alvar::AlvarMarker& closest_marker);

  /**
   * Return spotted markers satisfying the constraints specified by the parameters
   * @param younger_than    Elapsed time between now and markers timestamp must be below this limit.
   * @param min_confidence
   * @param exclude_globals
   * @param spotted_markers
   * @return
   */
  bool spotted(double younger_than, int min_confidence, bool exclude_globals,
               ar_track_alvar::AlvarMarkers& spotted_markers);
  bool closest(double younger_than, int min_confidence, bool exclude_globals,
               ar_track_alvar::AlvarMarker& closest_marker);

  bool spotDockMarker(uint32_t base_marker_id);

  static bool enableTracker();
  static bool disableTracker();

private:
  std::string global_frame_;
  std::string odom_frame_;
  std::string base_frame_;

  std::vector<uint32_t> times_spotted_;

  tf::TransformListener    tf_listener_;
  tf::TransformBroadcaster tf_brcaster_;
  double                   tf_brc_freq_;  /**< Allows enabling tf broadcasting; mostly for debug */

  ar_track_alvar::AlvarMarker  docking_marker_;  /**< AR markers described in the semantic map */
  ar_track_alvar::AlvarMarkers global_markers_;  /**< AR markers described in the semantic map */
  ar_track_alvar::AlvarMarkers spotted_markers_;

  ros::Subscriber tracked_markers_sub_;
  ros::Subscriber global_markers_sub_;

  boost::function<void (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&)>  robot_pose_cb_;
  boost::function<void (const geometry_msgs::PoseStamped::ConstPtr&, uint32_t)>    base_spotted_cb_;

  void broadcastMarkersTF();
  void globalMarkersCB(const ar_track_alvar::AlvarMarkers::ConstPtr& msg);
  void arPoseMarkersCB(const ar_track_alvar::AlvarMarkers::ConstPtr& msg);

  bool included(const uint32_t id, const ar_track_alvar::AlvarMarkers& v,
                ar_track_alvar::AlvarMarker* e = NULL)
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

  bool excluded(const uint32_t id, const ar_track_alvar::AlvarMarkers& v)
  {
    for (unsigned int i = 0; i < v.markers.size(); i++)
    {
      if (id == v.markers[i].id)
        return false;
    }

    return true;
  }
};

} /* namespace waiterbot */

#endif /* AR_MARKERS_HPP_ */
