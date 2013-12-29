/*
 * tf_handlers.cpp
 *
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/kobuki-x/license/LICENSE
 *
 *  Created on: Apr 6, 2013
 *      Author: jorge
 *  Modified on: Dec, 2013 
 *      Jihoon Lee
 */

#include "waiterbot_sensors/ar_marker_processor.hpp"

namespace waiterbot {

  void ARMarkerProcessor::broadcastMarkersTF()
  {
     // TODO semantic map is not broadcasting this already?  only docking base no
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


    for (unsigned int i = 0; i <global_markers_mirrors_.markers.size(); i++)
    {
      sprintf(child_frame, "%s_%d", i != docking_marker_.id?"MIRROR":"MIRROR docking", i);
      mtk::pose2tf(global_markers_mirrors_.markers[i].pose, tf);
      tf.child_frame_id_ = child_frame;
      tf.stamp_ = ros::Time::now();
      tf_brcaster_.sendTransform(tf);
    }


    // TODO remove definitively or recover id I finally decide not to include docking base on global markers
    // CHANGED  look below
    /*
    if (docking_marker_.id != std::numeric_limits<uint32_t>::max())
    {
      sprintf(child_frame, "docking_base_%d", docking_marker_.id);
      mtk::pose2tf(docking_marker_.pose, tf);
      tf.child_frame_id_ = child_frame;
      tf.stamp_ = ros::Time::now();
      tf_brcaster_.sendTransform(tf);
    }*/
  }

  bool ARMarkerProcessor::getMarkerTf(const std::string& ref_frame, uint32_t marker_id, const ros::Time& timestamp, tf::StampedTransform& tf)
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
}
