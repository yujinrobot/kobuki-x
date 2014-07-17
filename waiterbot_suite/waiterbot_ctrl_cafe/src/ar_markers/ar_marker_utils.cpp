
#include "waiterbot_ctrl_cafe/ar_markers.hpp"

namespace waiterbot
{

void ARMarkersCafe::createMirrorMarkers()
{
  for (unsigned int i = 0; i < global_markers_.markers.size(); i++)
  {
    // Compensate the vertical alignment of markers and put at ground level to adopt navistack goals format
    tf::Transform tf(tf::createQuaternionFromYaw(tf::getYaw(global_markers_.markers[i].pose.pose.orientation) - M_PI/2.0),
                     tf::Vector3(global_markers_.markers[i].pose.pose.position.x, global_markers_.markers[i].pose.pose.position.y, 0.0));
    tf::StampedTransform marker_gb(tf, ros::Time::now(), global_frame_, "KKKK");

    // Half turn and translate to put goal at some distance in front of the marker
    tf::Transform in_front(tf::createQuaternionFromYaw(M_PI),
                           tf::Vector3(1.0, 0.0, 0.0));
    marker_gb *= in_front;

    ar_track_alvar_msgs::AlvarMarker kk;
    mtk::tf2pose(marker_gb, kk.pose);
    global_markers_mirrors.markers.push_back(kk);

    ROS_DEBUG("Marker %d: %s", global_markers_.markers[i].id, mtk::pose2str(global_markers_.markers[i].pose.pose));
  }
}

void ARMarkersCafe::createFixedMarkers()
{
  global_markers_fix_ = global_markers_;
  for(unsigned int i=0; i < global_markers_.markers.size(); i++)
  {
    ar_track_alvar_msgs::AlvarMarker m = global_markers_.markers[i];
    m.id = m.id - 3;
    global_markers_fix_.markers.push_back(m);
  }
}

bool ARMarkersCafe::spotted(double younger_than, double min_confidence, ar_track_alvar_msgs::AlvarMarkers& spotted_markers)
{
  return ARMarkerTracking::spotted(younger_than, min_confidence, spotted_markers);
}
bool ARMarkersCafe::closest(double younger_than, double min_confidence, ar_track_alvar_msgs::AlvarMarker& closest_marker)
{
  return ARMarkerTracking::closest(younger_than, min_confidence, closest_marker);
}
bool ARMarkersCafe::spotted(double younger_than, const ar_track_alvar_msgs::AlvarMarkers& including, const ar_track_alvar_msgs::AlvarMarkers& excluding, ar_track_alvar_msgs::AlvarMarkers& spotted_markers)
{
  return ARMarkerTracking::spotted(younger_than, including, excluding, spotted_markers);
}
bool ARMarkersCafe::closest(const ar_track_alvar_msgs::AlvarMarkers& including, const ar_track_alvar_msgs::AlvarMarkers& excluding, ar_track_alvar_msgs::AlvarMarker& closest_marker)
{
  return ARMarkerTracking::closest(including, excluding, closest_marker);
}

bool ARMarkersCafe::spotted(double younger_than, double min_confidence, bool exclude_globals, ar_track_alvar_msgs::AlvarMarkers& spotted)
{
  ROS_INFO("Spotted MArker Size = %d",(int)spotted_markers_.markers.size());
  if (spotted_markers_.markers.size() == 0)
    return false;

  if ((ros::Time::now() - spotted_markers_.markers[0].header.stamp).toSec() >= younger_than)
  {
    // We must check the timestamp from an element in the markers list, as the one on message's header is always zero!
    // WARNING: parameter younger_than must be high enough, as ar_track_alvar_msgs publish at Kinect rate but only updates
    // timestamps about every 0.1 seconds (and now we can set it to run slower, as frequency is a dynamic parameter!)
    ROS_WARN("Spotted markers too old:   %f  >=  %f",   (ros::Time::now() - spotted_markers_.markers[0].header.stamp).toSec(), younger_than);
    return false;
  }

  spotted.header = spotted_markers_.header;
  spotted.markers.clear();
  for (unsigned int i = 0; i < spotted_markers_.markers.size(); i++)
  {
    ROS_INFO("Id = %d confidence : %.2f", spotted_markers_.markers[i].id, tracked_markers_[spotted_markers_.markers[i].id].confidence); 
    if ((exclude_globals == true) && (included(spotted_markers_.markers[i].id, global_markers_fix_) == true)) {
      ROS_INFO("Excluded");
      continue; 
    }

    //if (tracked_markers_[spotted_markers_.markers[i].id].confidence >= min_confidence)
    //{
    spotted.markers.push_back(spotted_markers_.markers[i]);
    //}
    break;
  }

  return (spotted.markers.size() > 0);
}

bool ARMarkersCafe::closest(double younger_than, double min_confidence, bool exclude_globals, ar_track_alvar_msgs::AlvarMarker& closest)
{
  ar_track_alvar_msgs::AlvarMarkers spotted_markers;
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

bool ARMarkersCafe::enableTracker()
{
  // There were many comments about enable/disable tracker. See old implementation if necessary
  return true;
}

bool ARMarkersCafe::disableTracker()
{
  return true;
}


void ARMarkersCafe::setRobotPoseCB(boost::function<void (const geometry_msgs::PoseWithCovarianceStamped::ConstPtr&)> cb)
{
  robot_pose_cb_ = cb;
}

void ARMarkersCafe::baseSpottedCB(boost::function<void (const geometry_msgs::PoseStamped::ConstPtr&, uint32_t)> cb)   
{
  base_spotted_cb_ = cb;
}

bool ARMarkersCafe::dockingBaseSpotted()
{
  return (docking_marker_.id != std::numeric_limits<uint32_t>::max()); 
}

const geometry_msgs::PoseStamped& ARMarkersCafe::getDockingBasePose() 
{ 
  return docking_marker_.pose; 
}
}
