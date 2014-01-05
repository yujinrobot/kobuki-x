/*
 * ar_marker_processor.hpp
 *
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/kobuki-x/license/LICENSE
 *
 *  Created on: Apr 6, 2013
 *      Author: jorge
 *  Modified on: Dec, 2013 
 *      Jihoon Lee          
 *
 */

#ifndef AR_PAIR_TRACKING_HPP_
#define AR_PAIR_TRACKING_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <yocs_math_toolkit/common.hpp>
#include <yocs_math_toolkit/geometry.hpp>
#include <ar_track_alvar/AlvarMarkers.h>
#include <waiterbot_ar_marker_tracking/tracking.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace waiterbot
{

namespace ARPairTrackingDefaultParams {
  const int    VENDING_MARKER_LEFT_ID    = 3;
  const int    VENDING_MARKER_RIGHT_ID   = 0;
  const double VENDING_MARKER_BASELINE       = 0.26;
  const std::string PUB_ROBOT_POSE_AR    = "robot_pose_ar";
}

/*****************************************************************************
** Classes
*****************************************************************************/

class ARPairTracking : public ARMarkerTracking
{
  public:

    ARPairTracking();
    virtual ~ARPairTracking();

    bool init();

  protected:
    // raw list of ar markers from ar_alvar_track pkg 
    void customCB(const ar_track_alvar::AlvarMarkers& spotted_markers, const std::vector<TrackedMarker> &tracked_markers);
    void computeRelativeRobotPose(const ar_track_alvar::AlvarMarkers& spotted_markers, const std::vector<TrackedMarker>& tracked_markers);

//    //////////////////// tf related
//    void broadcastMarkersTF();
//    bool getMarkerTf(const std::string& ref_frame, uint32_t marker_id, const ros::Time& timestamp, tf::StampedTransform& tf);

  private:
    // Confidence evaluation attributes
    // Other attributes
//    std::string global_frame_;
//    std::string odom_frame_;
//    std::string base_frame_;

    ros::Publisher     pub_robot_pose_ar_;

    tf::Transformer          tf_internal_;
    tf::TransformListener    tf_listener_;
    tf::TransformBroadcaster tf_brcaster_;
//    double                   tf_broadcast_freq_;  /**< Allows enabling tf broadcasting; mostly for debug */

    int                      vending_marker_left_id_;
    int                      vending_marker_right_id_;
    double                   vending_marker_baseline_;
};

} /* namespace waiterbot */

#endif /* AR_MARKERS_HPP_ */
