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

#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ar_track_alvar/AlvarMarkers.h>

namespace waiterbot
{

class ARMarkers
{
public:
  ARMarkers();
  virtual ~ARMarkers();

  bool init();

  void arPoseMarkerCB(const ar_track_alvar::AlvarMarkers::Ptr& msg);

  void setRobotPoseCB(boost::function<void (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&)> cb)
  {
    robot_pose_cb_ = cb;
  }

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
private:
  std::string global_frame_;
  std::string odom_frame_;
  std::string base_frame_;

  ros::Subscriber ar_pose_sub_;

  boost::function<void (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&)> robot_pose_cb_;

  std::vector<uint32_t> times_spotted_;

  uint32_t base_marker_id_;

  tf::TransformListener tf_listener_;

  ar_track_alvar::AlvarMarker  docking_marker_;  /**< AR markers described in the semantic map */
  ar_track_alvar::AlvarMarkers global_markers_;  /**< AR markers described in the semantic map */
  ar_track_alvar::AlvarMarkers spotted_markers_;

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

  double distance(geometry_msgs::Point a, geometry_msgs::Point b)
  {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
  }
};

} /* namespace waiterbot */

#endif /* AR_MARKERS_HPP_ */
