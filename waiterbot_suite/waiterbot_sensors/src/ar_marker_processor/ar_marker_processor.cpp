/*
 * ar_marker_processor.hpp
 *
 *  LICENSE : BSD - https://raw.github.com/yujinrobot/kobuki-x/license/LICENSE
 *
 *  Created on: Apr 6, 2013
 *      Author: jorge
 *  Modified on: Dec, 2013 
 *      Jihoon Lee
 */

#include "waiterbot_sensors/ar_marker_processor.hpp"

namespace waiterbot
{

ARMarkerProcessor::ARMarkerProcessor() { 
  init(); 
}

ARMarkerProcessor::~ARMarkerProcessor() {}

bool ARMarkerProcessor::init()
{
  ros::NodeHandle nh, pnh("~");
  
  // Parameters
  pnh.param("global_frame",       global_frame_,           ARMarkerProcessorDefaultParams::GLOBAL_FRAME);
  pnh.param("odom_frame",         odom_frame_,             ARMarkerProcessorDefaultParams::ODOM_FRAME); 
  pnh.param("base_frame",         base_frame_,             ARMarkerProcessorDefaultParams::BASE_FRAME); 
  pnh.param("tf_broadcast_freq",  tf_broadcast_freq_,      ARMarkerProcessorDefaultParams::TF_BROADCAST_FREQ); // disabled by default
  pnh.param("global_pose_conf",   global_pose_conf_,       ARMarkerProcessorDefaultParams::GLOBAL_POSE_CONF);
  pnh.param("docking_base_conf",  docking_base_conf_,      ARMarkerProcessorDefaultParams::DOCKING_BASE_CONF);
  pnh.param("max_valid_d_inc",    max_valid_d_inc_,        ARMarkerProcessorDefaultParams::MAX_VALID_D_INC);
  pnh.param("max_valid_h_inc",    max_valid_h_inc_,        ARMarkerProcessorDefaultParams::MAX_VALID_H_INC);
  pnh.param("max_tracking_time",  max_tracking_time_,      ARMarkerProcessorDefaultParams::MAX_TRACKING_TIME);
  pnh.param("min_penalized_dist", min_penalized_dist_,     ARMarkerProcessorDefaultParams::MIN_PENALIZED_DIST);
  pnh.param("max_reliable_dist",  max_reliable_dist_,      ARMarkerProcessorDefaultParams::MAX_RELIABLE_DIST);
  pnh.param("min_penalized_head", min_penalized_head_,     ARMarkerProcessorDefaultParams::MIN_PENALIZED_HEAD);
  pnh.param("max_reliable_head",  max_reliable_head_,      ARMarkerProcessorDefaultParams::MAX_RELIABLE_HEAD);
  pnh.param("docking_marker_id",  docking_marker_id_,      ARMarkerProcessorDefaultParams::DOCKING_MARKER_ID);
  pnh.param("vending_marker_left_id",  vending_marker_left_id_,   ARMarkerProcessorDefaultParams::VENDING_MARKER_LEFT_ID); 
  pnh.param("vending_marker_right_id", vending_marker_right_id_, ARMarkerProcessorDefaultParams::VENDING_MARKER_RIGHT_ID); 
  pnh.param("vending_marker_dist",     vending_marker_dist_,       ARMarkerProcessorDefaultParams::VENDING_MARKER_DIST); 

  global_marker_localization_ = false;
  if(pnh.getParam("global_marker_filename",        global_marker_filename_) == false)
  {
    ROS_ERROR("global marker filename parameter is not set!. Cannot use ar_based localization");
  }

  if (nh.getParam("ar_track_alvar/max_frequency", ar_tracker_freq_) == false)
  {
    ar_tracker_freq_ = ARMarkerProcessorDefaultParams::AR_TRACKER_FREQ;
    ROS_WARN("Cannot get AR tracker frequency; using default value (%f)", ar_tracker_freq_);
    ROS_WARN("Confidence evaluation can get compromised if this is not the right value!");
  }

  sub_ar_markers_ = nh.subscribe("ar_track_alvar/ar_pose_marker", 1, &ARMarkerProcessor::arPoseMarkersCB, this);

  pub_robot_pose_ar_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(ARMarkerProcessorDefaultParams::PUB_ROBOT_POSE_AR, 1);
  pub_dock_pose_ar_  = nh.advertise<geometry_msgs::PoseStamped>(ARMarkerProcessorDefaultParams::PUB_DOCK_POSE_AR, 1);

  // There are 18 different markers
  tracked_markers_.resize(ARMarkerProcessorDefaultParams::MARKERS_COUNT);

  return true;
}

void ARMarkerProcessor::loadGlobalMarkers()
{
  if(loadAlvarMarkersFromYaml(global_marker_filename_, global_markers_) == false)
  {
    ROS_WARN("Cannot use global marker based localization");
    global_marker_localization_ = false;
    return;
  }

  // Just take first message; ignore the rest, as global markers list is not dynamic
  ROS_INFO("%lu global marker pose(s) received", global_markers_.markers.size());
  for (unsigned int i = 0; i < global_markers_.markers.size(); i++)
  {
    // Compensate the vertical alignment of markers and put at ground level to adopt navistack goals format
    tf::Transform tf(tf::createQuaternionFromYaw(tf::getYaw(global_markers_.markers[i].pose.pose.orientation) - M_PI/2.0),
                     tf::Vector3(global_markers_.markers[i].pose.pose.position.x, global_markers_.markers[i].pose.pose.position.y, 0.0));
    tf::StampedTransform marker_gb(tf, ros::Time::now(), global_frame_, "global_marker_frame");

    // Half turn and translate to put goal at some distance in front of the marker
    tf::Transform in_front(tf::createQuaternionFromYaw(M_PI),
                           tf::Vector3(1.0, 0.0, 0.0));
    marker_gb *= in_front;

    ar_track_alvar::AlvarMarker kk;
    mtk::tf2pose(marker_gb, kk.pose);
    global_markers_mirrors_.markers.push_back(kk);

    ROS_DEBUG("Marker %d: %s", global_markers_.markers[i].id, mtk::pose2str(global_markers_.markers[i].pose.pose));
  }

  global_marker_localization_ = true;
}

void ARMarkerProcessor::arPoseMarkersCB(const ar_track_alvar::AlvarMarkers::ConstPtr& msg)
{
  // TODO: use confidence to incorporate covariance to global poses
  //ROS_INFO("Received msg");

  // Maintain markers
  maintainTrackedMarkers(msg, tracked_markers_);

//    processDockingMarkers(msg, tracked_markers_);
  processGlobalMarkers(msg, tracked_markers_);
    
  computeRelativeRobotPose(msg, tracked_markers_);

  spotted_markers_ = *msg;
}

void ARMarkerProcessor::maintainTrackedMarkers(const ar_track_alvar::AlvarMarkers::ConstPtr& msg,std::vector<TrackedMarker>& tracked_markers)
{
  // Make thresholds relative to the tracking frequency (as it can be dynamically changed)
  int obs_list_max_size  = (int)round(max_tracking_time_*ar_tracker_freq_);
  double max_valid_d_inc = max_valid_d_inc_ / ar_tracker_freq_;
  double max_valid_h_inc = max_valid_h_inc_ / ar_tracker_freq_;

  for (unsigned int i = 0; i < msg->markers.size(); i++)
  {
    if (msg->markers[i].id >= tracked_markers.size())
    {
      // A recognition error from Alvar markers tracker
      ROS_WARN("Discarding AR marker with unrecognized id (%d)", msg->markers[i].id);
      continue;
    }
    //ROS_INFO("Maintaining marker id = %d",msg->markers[i].id);

    TrackedMarker& marker = tracked_markers[msg->markers[i].id];

    // Confidence evaluation
    maintainTrackedMarker(marker, msg->markers[i], obs_list_max_size, max_valid_d_inc, max_valid_h_inc);
  }
}


void ARMarkerProcessor::maintainTrackedMarker(TrackedMarker& marker,const ar_track_alvar::AlvarMarker& msgMarker, const int obs_list_max_size, const double max_valid_d_inc, const double max_valid_h_inc)
{
  marker.distance = mtk::distance3D(msgMarker.pose.pose);
  marker.distance2d = mtk::distance2D(msgMarker.pose.pose.position.x, msgMarker.pose.pose.position.z, 0,0);
  marker.heading  = tf::getYaw(msgMarker.pose.pose.orientation) + M_PI/2.0;
  // WARN: note that heading evaluation also assumes vertically aligned markers
  //ROS_INFO("Marker dist = %.4f",marker.distance2d);
  //ROS_INFO("Marer  head = %.4f",marker.heading);

  int position = 0;
  ros::Time now = ros::Time::now();
  geometry_msgs::PoseStamped prev = msgMarker.pose;
  for (ObsList::iterator it = marker.obs_list_.begin(); it != marker.obs_list_.end(); ++it)
  {
    double age = (now - it->header.stamp).toSec();
    if (age > ((position + 1)/ar_tracker_freq_) + 1.0)
    {
      int s0 = marker.obs_list_.size();
      marker.obs_list_.erase(it, marker.obs_list_.end());
      int s1 = marker.obs_list_.size();
      ROS_INFO("%d observations discarded (first one with position %d in the list) for being %f seconds old ( > %f)", s0 - s1, position, age, ((position + 1)/ar_tracker_freq_) + 1.0);
      break;
    }

    if ((mtk::distance3D(prev.pose, it->pose) > max_valid_d_inc) || (std::abs(mtk::minAngle(prev.pose, it->pose)) > max_valid_h_inc))
    {
      // Incoherent observation; stop going over the list and use current position value to fill confidence values
      ROS_ERROR("%d  BREAK at %d   %f  %f     %f   %f        %f", msgMarker.id, position, mtk::distance3D(prev.pose, it->pose),  mtk::minAngle(prev.pose, it->pose), max_valid_d_inc, max_valid_h_inc, ar_tracker_freq_);
      break;
    }

    prev = *it;
    position++;
  }

  marker.conf_distance = marker.distance <= min_penalized_dist_ ? 1.0
                       : marker.distance >= max_reliable_dist_  ? 0.0
                       : 1.0 - std::pow((marker.distance - min_penalized_dist_) / (max_reliable_dist_ - min_penalized_dist_), 2);
  marker.conf_heading  = std::abs(marker.heading) <= min_penalized_head_ ? 1.0
                       : std::abs(marker.heading) >= max_reliable_head_  ? 0.0
                       : 1.0 - std::pow((std::abs(marker.heading) - min_penalized_head_) / (max_reliable_head_ - min_penalized_head_), 2);
  marker.stability     = marker.obs_list_.size() == 0 ? 0.0 : std::sqrt((double)position/(double)marker.obs_list_.size());
  marker.persistence   = std::sqrt((double)marker.obs_list_.size() / (double)obs_list_max_size);
  marker.confidence    = marker.conf_distance * marker.conf_heading * marker.stability * marker.persistence;
  marker.obs_list_.insert(marker.obs_list_.begin(), msgMarker.pose);
  marker.obs_list_.begin()->header = msgMarker.header;  // bloody alvar tracker doesn't fill pose's header
  if (marker.obs_list_.size() > obs_list_max_size)
    marker.obs_list_.pop_back();

  /*
      ROS_INFO_STREAM(msgMarker.id << ":  Dist : "
                   << marker.distance2d << " Heading : [" << marker.heading << "] Confidence : [" << marker.confidence << "]   "
                    << marker.conf_distance << "   " << marker.conf_heading << " Stability : ["
                    << marker.stability << "]   ["<< marker.persistence << "]   [" << position << "]   " << marker.obs_list_.size());
                    */
}

void ARMarkerProcessor::processDockingMarkers(const ar_track_alvar::AlvarMarkers::ConstPtr& msg,std::vector<TrackedMarker>& tracked_markers)
{
  /*
    // Check if the marker is docking marker
    if ((unsigned int)(msg->markers[i].id) == (unsigned int)docking_marker_id_)
    {
      // This is the docking base marker! call the registered callbacks if it's reliable enough
      if (marker.confidence <= docking_base_conf_)
      {
        ROS_WARN_THROTTLE(1, "Ignoring docking base marker at it is not reliable enough (%f < %f)",
                              marker.confidence, docking_base_conf_);
        continue;
      }

      boost::shared_ptr<geometry_msgs::PoseStamped> ps(new geometry_msgs::PoseStamped());
      *ps = msg->markers[i].pose;
      ps->header = msg->markers[i].header;  // bloody alvar tracker doesn't fill pose's header
      pub_dock_pose_ar_.publish(ps);
      base_spotted_cb_(ps, msg->markers[i].id);
    }
    */
}

void ARMarkerProcessor::processGlobalMarkers(const ar_track_alvar::AlvarMarkers::ConstPtr& msg,std::vector<TrackedMarker>& tracked_markers)
{
  // compare with detected global markers and recorded ar markers position. 
  // confirm both are located good.
  
  // if both are good, measure robot's relative pose with recorded ar marker pose.




  // send initial pose 

  // send pose of  
  for (unsigned int i = 0; i < msg->markers.size(); i++)
  {
    ar_track_alvar::AlvarMarker global_marker;
    TrackedMarker& marker = tracked_markers[msg->markers[i].id];

    if (included(msg->markers[i].id, global_markers_, &global_marker) == true) // if it is global marker
    {
      // This is a global marker! infer robot's global pose and call registered callbacks if it's reliable enough
      if (marker.confidence <= global_pose_conf_)
      {
        //if (msg->markers[i].id == 1)
        ROS_DEBUG_THROTTLE(1, "Discarding global marker at it is not reliable enough (%f < %f)", marker.confidence, global_pose_conf_);
      }
      else 
      {
        processGlobalMarker(marker, msg->markers[i], global_marker);
      }
    }
  }
}


void ARMarkerProcessor::processGlobalMarker(const TrackedMarker& marker, const ar_track_alvar::AlvarMarker& msgMarker,const ar_track_alvar::AlvarMarker& global_marker)
{
   boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped> pwcs(new geometry_msgs::PoseWithCovarianceStamped);

   // Marker tf on global reference system
   tf::StampedTransform marker_gb;
   mtk::pose2tf(global_marker.pose, marker_gb);

   // Marker tf on robot base reference system
   tf::StampedTransform marker_bs;
   if (getMarkerTf(base_frame_, msgMarker.id, msgMarker.header.stamp, marker_bs) == false)
   {
     // This should not happen unless AR tracker made a mistake, as we are using the timestamp he reported
     ROS_ERROR("Global marker spotted but we failed to get its tf");
     return;
   }

   // Calculate robot tf on global reference system multiplying the global marker tf (known a priori)
   // by marker tf on base reference system, that is, "subtract" the relative tf to the absolute one
   tf::Transform robot_gb = marker_gb * marker_bs.inverse();
   mtk::tf2pose(robot_gb, pwcs->pose.pose);

   pwcs->header.stamp = msgMarker.header.stamp;
   pwcs->header.frame_id = global_frame_;

   // publish robot pose to nav watch dog
   pub_robot_pose_ar_.publish(pwcs);
}

void ARMarkerProcessor::computeRelativeRobotPose(const ar_track_alvar::AlvarMarkers::ConstPtr& msg,std::vector<TrackedMarker>& tracked_markers)
{

  ar_track_alvar::AlvarMarker left;
  ar_track_alvar::AlvarMarker right;

  if((included(vending_marker_left_id_, *msg, &left) && included(vending_marker_right_id_,*msg, &right))
      &&
      tracked_markers[vending_marker_left_id_].confidence > global_pose_conf_ &&
      tracked_markers[vending_marker_right_id_].confidence > global_pose_conf_)
  {

    double left_side = tracked_markers[vending_marker_left_id_].distance2d;
    double right_side = tracked_markers[vending_marker_right_id_].distance2d;

    double left_x = left.pose.pose.position.x;
    double left_z = left.pose.pose.position.z;
    double right_x = right.pose.pose.position.x;
    double right_z = right.pose.pose.position.z;

    // angle between the robot and the first marker
    double alpha = atan2(left_x, left_z);
    double alpha_degrees = alpha * (180.0) / M_PI;

    // alpah + beta is angle between the robot and the second marker
    double beta = atan2(right_x, right_z);
    double beta_degrees = beta * (180.0) / M_PI;

    // theta is the angle between the wall and the perpendicular in front of the robot
    double theta = atan2((left_z - right_z), (right_x - left_x));
    double theta_degrees = theta * (180.0) / M_PI;

    double target_x = left_x + (right_x - left_x) / 2 - 0.4 * sin(theta);
    double target_z = left_z + (right_z - left_z) / 2 - 0.4 * cos(theta);
    double target_heading = atan2(target_x, target_z);
    double target_heading_degrees = target_heading * 180.0 / M_PI;

    /*
    ROS_INFO("Alpha = %.3f",alpha_degrees);
    ROS_INFO("Beta  = %.3f",beta_degrees);
    ROS_INFO("Theta = %.3f",theta_degrees);
    */
    //ROS_INFO("Target (x = %.3f, z = %.3f, heading = %.3f)", target_x, target_z, target_heading_degrees);

    // target_pose -> robot
//    std::string frame = "robot";
    std::string frame = "camera_rgb_optical_frame";
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "target_pose";
    pose.pose.position.x = -target_x;  
    pose.pose.position.y = 0; 
    pose.pose.position.z = target_z;

    tf::Quaternion quat(target_heading,0,0);
    quat *= tf::Quaternion(0,M_PI,0);
 
    pose.pose.orientation.x = quat.getX(); 
    pose.pose.orientation.y = quat.getY(); 
    pose.pose.orientation.z = quat.getZ(); 
    pose.pose.orientation.w = quat.getW(); 

//    tf::StampedTransform tf;
//    tf.child_frame_id_ = frame;
//    mtk::pose2tf(pose, tf);
//    tf.stamp_ = ros::Time::now();
//    tf_brcaster_.sendTransform(tf);

    try
    {
      // get and set ar_link -> target_pose
      tf::StampedTransform tf_ar_target_pose;
      tf_listener_.lookupTransform("ar_global", "target_pose", ros::Time(0), tf_ar_target_pose);
      tf_internal_.setTransform(tf_ar_target_pose);
      /*
      ROS_INFO_STREAM("ar_link -> target_position: x = " << tf_ar_target_pose.getOrigin().x()
                     << ", y = " << tf_ar_target_pose.getOrigin().y()
                     << ", z = " << tf_ar_target_pose.getOrigin().z());
                     */

      // set target_pose -> camera
      tf::StampedTransform tf_ar_camera;
      tf_ar_camera.child_frame_id_ = frame;
      mtk::pose2tf(pose, tf_ar_camera);
      tf_ar_camera.stamp_ = ros::Time::now();
      tf_internal_.setTransform(tf_ar_camera);
      /*
      ROS_INFO_STREAM("target_pose -> camera_rgb_optical_frame: x = " << tf_ar_camera.getOrigin().x()
                     << ", y = " << tf_ar_camera.getOrigin().y()
                     << ", z = " << tf_ar_camera.getOrigin().z());
                     */

      // get and set camera -> base_footprint 
      tf::StampedTransform tf_camera_base_footprint;
      tf_listener_.lookupTransform("camera_rgb_optical_frame", "base_footprint", ros::Time(0), tf_camera_base_footprint);
      tf_internal_.setTransform(tf_camera_base_footprint);
      /*
      ROS_INFO_STREAM("camera_rgb_optical_frame -> odom: x = " << tf_camera_odom.getOrigin().x()
                     << ", y = " << tf_camera_odom.getOrigin().y()
                     << ", z = " << tf_camera_odom.getOrigin().z());
                     */

      // get and publish ar_link -> base_footprint
      tf::StampedTransform tf_ar_base_footprint;
      tf_internal_.lookupTransform("base_footprint", "ar_global", ros::Time(0), tf_ar_base_footprint);
      boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped> pwcs(new geometry_msgs::PoseWithCovarianceStamped);

      pwcs->header.stamp = tf_ar_base_footprint.stamp_;
      pwcs->header.frame_id = "ar_global";

      geometry_msgs::PoseStamped ps;
      mtk::tf2pose(tf_ar_base_footprint, ps);
      pwcs->pose.pose = ps.pose;

      // publish robot pose to nav watch dog
      pub_robot_pose_ar_.publish(pwcs);



      //tf_brcaster_.sendTransform(tf_ar_odom);
      /*
      ROS_INFO_STREAM("ar_link -> odom: x = " << tf_ar_odom.getOrigin().x()
                     << ", y = " << tf_ar_odom.getOrigin().y()
                     << ", z = " << tf_ar_odom.getOrigin().z());
                     */
    }
    catch (tf::TransformException const &ex)
    {
      ROS_WARN_STREAM("TF error: " << ex.what());
      ROS_INFO_STREAM("All known frames: " << tf_internal_.allFramesAsString());      
    }









  }
}

void ARMarkerProcessor::spin()
{
  loadGlobalMarkers();
  ROS_INFO("AR Marker Processer initialized");

  // Broadcasts the spotted ar markers tf

  if (tf_broadcast_freq_ > 0.0)
  {
    ros::Rate rate(tf_broadcast_freq_);

    while(ros::ok())
    {
      broadcastMarkersTF();
      rate.sleep();
      ros::spinOnce();
    }
  }                                                                 
  else {
    while(ros::ok())
    {
      ros::Duration(0.5).sleep();
      ros::spinOnce();
    }
  }
}

} // waiterbot namespace
