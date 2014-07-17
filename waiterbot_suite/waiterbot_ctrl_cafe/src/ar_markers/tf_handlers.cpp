
#include "waiterbot_ctrl_cafe/ar_markers.hpp"

namespace waiterbot 
{

void ARMarkersCafe::broadcastMarkersTF()
{
  ros::Rate rate(tf_broadcast_freq_);

  // TODO semantic map is not broadcasting this already?  only docking base no
  while (ros::ok())
  {
    publishMarkerTFs(global_marker_prefix_, global_markers_);
    publishTargetTFs(global_marker_prefix_, global_markers_);
    publishMarkerTFs(global_marker_mirror_prefix_, global_markers_mirrors);

// TODO remove definitively or recover id I finally decide not to include docking base on global markers
// CHANGED  look below
    if (docking_marker_.id != std::numeric_limits<uint32_t>::max())
    {
      char child_frame[32];
      tf::StampedTransform tf;
      tf.stamp_ = ros::Time::now();

      sprintf(child_frame, "docking_base_%d", docking_marker_.id);
      mtk::pose2tf(docking_marker_.pose, tf);
      tf.child_frame_id_ = child_frame;
      tf.stamp_ = ros::Time::now();
      tf_brcaster_.sendTransform(tf);
    }

    rate.sleep();
  }
}
  
void ARMarkersCafe::publishMarkerTFs(const std::string prefix, const ar_track_alvar_msgs::AlvarMarkers& markers)
{
  char child_frame[32];
  tf::StampedTransform tf;
  tf.stamp_ = ros::Time::now();

  for (unsigned int i = 0; i <markers.markers.size(); i++)
  {
    sprintf(child_frame, "%s_%d", prefix.c_str(), markers.markers[i].id);
    mtk::pose2tf(markers.markers[i].pose, tf);
    tf.child_frame_id_ = child_frame;
    tf.stamp_ = ros::Time::now();
    tf_brcaster_.sendTransform(tf);
  }
}

void ARMarkersCafe::publishTargetTFs(const std::string prefix, const ar_track_alvar_msgs::AlvarMarkers& markers)
{
  char parent_frame[32];
  char child_frame[32];
  tf::StampedTransform tf;
  tf.stamp_ = ros::Time::now();

  for (unsigned int i = 0; i <markers.markers.size(); i++)
  {
    sprintf(parent_frame, "%s_%d", prefix.c_str(), markers.markers[i].id);
    sprintf(child_frame, "%s_%s", parent_frame, target_frame_postfix_.c_str());

    tf::Transform tf(tf::createQuaternionFromRPY(M_PI,0,0), tf::Vector3(0, 0, target_offset_));
    tf::StampedTransform target(tf, ros::Time::now(), parent_frame, child_frame);
    tf_brcaster_.sendTransform(target);
  }
}

void ARMarkersCafe::publish_transform(const std::string parent, const std::string child,const tf::Transform& t)
{
  tf::StampedTransform tt(t, ros::Time::now(), parent, child);
  tf_brcaster_.sendTransform(tt);
}

void ARMarkersCafe::print_stampedtransform(const std::string& name, tf::StampedTransform& t)
{
  tf::Vector3 p = t.getOrigin();
  tf::Quaternion q = t.getRotation();
  ROS_INFO("%s : %.2f %.2f %.2f,  %.2f %.2f %.2f %.2f ", name.c_str(), p.getX(),p.getY(), p.getZ(), q.getX(), q.getY(), q.getZ(), q.getW());
}

void ARMarkersCafe::print_transform(const std::string& name, tf::Transform& t)
{
  tf::Vector3 p = t.getOrigin();
  tf::Quaternion q = t.getRotation();
  ROS_INFO("%s : %.2f %.2f %.2f,  %.2f %.2f %.2f %.2f ", name.c_str(), p.getX(),p.getY(), p.getZ(), q.getX(), q.getY(), q.getZ(), q.getW());
}

bool ARMarkersCafe::getMarkerTf(const std::string& ref_frame, const std::string& prefix, uint32_t marker_id,
                               const ros::Time& timestamp, tf::StampedTransform& tf, const float timeout)
{
  char marker_frame[32];
  sprintf(marker_frame, "%s_%d", prefix.c_str(), marker_id);
  std::string target_frame(marker_frame);

  return getTf(ref_frame, target_frame, timestamp, tf, timeout);
}

bool ARMarkersCafe::getTf(const std::string& ref_frame, const std::string& marker_frame, 
                               const ros::Time& timestamp, tf::StampedTransform& tf, const float timeout)
{
  try
  {
    // Get marker tf on given reference system
    tf_listener_.waitForTransform(ref_frame, marker_frame, timestamp, ros::Duration(timeout));
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
    ROS_ERROR("Cannot get tf %s -> %s: %s", ref_frame.c_str(), marker_frame.c_str(), e.what());
    return false;
  }
  return true;
}

}
