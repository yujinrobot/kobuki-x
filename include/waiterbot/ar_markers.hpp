/*
 * ar_markers.hpp
 *
 *  Created on: Apr 6, 2013
 *      Author: jorge
 */

#ifndef AR_MARKERS_HPP_
#define AR_MARKERS_HPP_

#include <ar_track_alvar/AlvarMarkers.h>

namespace waiterbot
{

class ARMarkers
{
public:
  ARMarkers();
  virtual ~ARMarkers();

  void arPoseMarkerCB(const ar_track_alvar::AlvarMarkersConstPtr& msg);

  bool spotted(double younger_than,
               const ar_track_alvar::AlvarMarkers& including, const ar_track_alvar::AlvarMarkers& excluding,
               ar_track_alvar::AlvarMarkers& spotted_markers);
  bool closest(const ar_track_alvar::AlvarMarkers& including, const ar_track_alvar::AlvarMarkers& excluding,
               ar_track_alvar::AlvarMarker& closest_marker);

private:
  ar_track_alvar::AlvarMarkers markers_;

  static bool included(const uint32_t id, const ar_track_alvar::AlvarMarkers& v)
  {
    for (unsigned int i = 0; i < v.markers.size(); i++)
    {
      if (id == v.markers[i].id)
        return true;
    }

    return false;
  }

  static bool excluded(const uint32_t id, const ar_track_alvar::AlvarMarkers& v)
  {
    for (unsigned int i = 0; i < v.markers.size(); i++)
    {
      if (id == v.markers[i].id)
        return false;
    }

    return true;
  }

  static double distance(geometry_msgs::Point a, geometry_msgs::Point b)
  {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
  }
};

} /* namespace waiterbot */

#endif /* AR_MARKERS_HPP_ */
