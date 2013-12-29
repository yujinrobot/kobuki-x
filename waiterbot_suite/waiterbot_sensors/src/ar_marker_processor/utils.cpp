/*
 * ar_marker_processor.hpp
 *
 *  utility functions
 *
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/kobuki-x/license/LICENSE
 *
 *  Created on: Apr 6, 2013
 *      Author: jorge
 *  Modified on: Dec, 2013 
 *      Jihoon Lee          
 *
 */

#include "waiterbot_sensors/ar_marker_processor.hpp"

namespace waiterbot {
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
}
