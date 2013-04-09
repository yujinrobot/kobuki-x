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
  // TODO Auto-generated constructor stub

}

ARMarkers::~ARMarkers()
{
  // TODO Auto-generated destructor stub
}


void ARMarkers::arPoseMarkerCB(const ar_track_alvar::AlvarMarkersConstPtr& msg)
{
  // TODO MAke pointer!!!!  to avoid copyng    but take care of multi-threading
  markers_ = *msg;
}

bool ARMarkers::spotted(double younger_than,
                        const ar_track_alvar::AlvarMarkers& including,
                        const ar_track_alvar::AlvarMarkers& excluding,
                              ar_track_alvar::AlvarMarkers& spotted)
{
  if ((ros::Time::now() - markers_.header.stamp).toSec() >= younger_than)
  {
    return false;
  }

  spotted.header = markers_.header;
  spotted.markers.clear();
  for (unsigned int i = 0; i < markers_.markers.size(); i++)
  {
    if ((included(markers_.markers[i].id, including) == true) &&
        (excluded(markers_.markers[i].id, excluding) == true))
    {
      spotted.markers.push_back(markers_.markers[i]);
    }
  }

  return (spotted.markers.size() > 0);
}

bool ARMarkers::closest(const ar_track_alvar::AlvarMarkers& including,
                        const ar_track_alvar::AlvarMarkers& excluding,
                              ar_track_alvar::AlvarMarker& closest)
{
  double closest_dist = std::numeric_limits<double>::max();
  ar_track_alvar::AlvarMarker closest_marker;
  for (unsigned int i = 0; i < markers_.markers.size(); i++)
  {
    if ((included(markers_.markers[i].id, including) == true) &&
        (excluded(markers_.markers[i].id, excluding) == true))
    {
      double d = distance(markers_.markers[i].pose.pose.position, geometry_msgs::Point());
      if (d < closest_dist)
      {
        closest_dist = d;
        closest_marker = markers_.markers[i];
      }
    }
  }

  return (closest_dist < std::numeric_limits<double>::max());
}

} /* namespace waiterbot */
