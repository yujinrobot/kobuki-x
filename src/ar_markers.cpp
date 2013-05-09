/*
 * ar_markers.cpp
 *
 *  Created on: Apr 6, 2013
 *      Author: jorge
 */

#include "waiterbot/common.hpp"
#include "waiterbot/ar_markers.hpp"

namespace waiterbot
{

ARMarkers::ARMarkers()
{
  // Invalid id until we localize it globally
  docking_marker_.id = std::numeric_limits<uint32_t>::max();

/*
//TODO TEMPORAL FOR DEBUG

  docking_marker_.id = 6;
  docking_marker_.pose.header.frame_id = "map";
  docking_marker_.pose.pose.position.x = -0.1;
  docking_marker_.pose.pose.position.y = -1.9;
  docking_marker_.pose.pose.position.z = 0.1;
  docking_marker_.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI/2.0, 0.0, M_PI);
*/
}

ARMarkers::~ARMarkers()
{
}

bool ARMarkers::init()
{
  ros::NodeHandle nh, pnh("~");

  // Parameters
  pnh.param("tf_broadcast_freq", tf_brc_freq_, 0.0);  // disabled by default
  pnh.param("global_frame", global_frame_, std::string("map"));
  pnh.param("odom_frame",   odom_frame_,   std::string("odom"));
  pnh.param("base_frame",   base_frame_,   std::string("base_footprint"));

  tracked_markers_sub_ = nh.subscribe("ar_track_alvar/ar_pose_marker", 1, &ARMarkers::arPoseMarkersCB, this);
  global_markers_sub_  = nh.subscribe(             "marker_pose_list", 1, &ARMarkers::globalMarkersCB, this);

  // There are 18 different markers
  times_spotted_.resize(AR_MARKERS_COUNT, 0);

  if (tf_brc_freq_ > 0.0)
  {
    boost::thread(&ARMarkers::broadcastMarkersTF, this);
  }

  // Disable tracking until needed
  disableTracker();

  return true;
}

void ARMarkers::broadcastMarkersTF()
{
  ros::Rate rate(tf_brc_freq_);

  // TODO semantic map is not broadcasting this already?  only docking base no
  while (ros::ok())
  {
    char child_frame[32];
    tf::StampedTransform tf;
    tf.stamp_ = ros::Time::now();

    for (unsigned int i = 0; i <global_markers_.markers.size(); i++)
    {
      sprintf(child_frame, "%s_%d", i != docking_marker_.id?"global_marker":"docking_base", global_markers_.markers[i].id);
      tk::pose2tf(global_markers_.markers[i].pose, tf);
      tf.child_frame_id_ = child_frame;
      tf.stamp_ = ros::Time::now();
      tf_brcaster_.sendTransform(tf);
    }

// TODO remove definitively or recover id I finally decide not to include docking base on global markers
//    if (docking_marker_.id != std::numeric_limits<uint32_t>::max())
//    {
//      sprintf(child_frame, "docking_base_%d", docking_marker_.id);
//      tk::pose2tf(docking_marker_.pose, tf);
//      tf.child_frame_id_ = child_frame;
//      tf.stamp_ = ros::Time::now();
//      tf_brcaster_.sendTransform(tf);
//    }

    rate.sleep();
  }
}

void ARMarkers::globalMarkersCB(const ar_track_alvar::AlvarMarkers::ConstPtr& msg)
{
  // Just take first message; ignore the rest, as global markers list is not dynamic
  if ((global_markers_.markers.size() == 0) && (msg->markers.size() > 0))
  {
    global_markers_ = *msg;
    ROS_INFO("%lu global marker pose(s) received", global_markers_.markers.size());
    for (unsigned int i = 0; i < global_markers_.markers.size(); i++)
    {
      /*
      //TODO cheat for debuging;  remove
            if (global_markers_.markers[i].id >= 4)
            {
              ROS_DEBUG("Docking marker %d: %s", global_markers_.markers[i].id, tk::pose2str(global_markers_.markers[i].pose.pose));
              docking_marker_ = global_markers_.markers[i];
              global_markers_.markers.pop_back();
            }
            else

      */
      ROS_DEBUG("Marker %d: %s", global_markers_.markers[i].id, tk::pose2str(global_markers_.markers[i].pose.pose));
    }
  }
}

void ARMarkers::arPoseMarkersCB(const ar_track_alvar::AlvarMarkers::ConstPtr& msg)
{
  // TODO MAke pointer!!!!  to avoid copying    but take care of multi-threading
  // more TODO:  inc confidence is very shitty as quality measure ->  we need a filter!!!  >>>   and also incorporate on covariance!!!!

  for (unsigned int i = 0; i < msg->markers.size(); i++)
  {
    if (msg->markers[i].id >= times_spotted_.size())
    {
      // A recognition error from Alvar markers tracker
      ROS_WARN("Discarding AR marker with unrecognized id (%d)", msg->markers[i].id);
      continue;
    }

    times_spotted_[msg->markers[i].id] += 2;

    if ((msg->markers[i].id == docking_marker_.id) &&
        (times_spotted_[msg->markers[i].id] > 4))  // publish only with 3 or more spots
    {
      // This is the docking base marker! call the registered callbacks
      boost::shared_ptr<geometry_msgs::PoseStamped> ps(new geometry_msgs::PoseStamped());
      *ps = msg->markers[i].pose;
      ps->header = msg->markers[i].header;  // bloody alvar tracker doesn't fill pose's header
      base_spotted_cb_(ps, msg->markers[i].id);
    }

    ar_track_alvar::AlvarMarker global_marker;
    if ((included(msg->markers[i].id, global_markers_, &global_marker) == true) &&
        (times_spotted_[msg->markers[i].id] > 4))  // publish only with 3 or more spots
    {
      // This is a global marker! infer the robot's global pose and call the registered callbacks
      boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped> pwcs(new geometry_msgs::PoseWithCovarianceStamped);

      // Marker tf on global reference system
      tf::StampedTransform marker_gb;
      tk::pose2tf(global_marker.pose, marker_gb);

      // Marker tf on robot base reference system
      tf::StampedTransform marker_bs;
      if (getMarkerTf(base_frame_, msg->markers[i].id, msg->markers[i].header.stamp, marker_bs) == false)
      {
        // This should not happen unless AR tracker made a mistake, as we are using the timestamp he reported
        ROS_ERROR("Global marker spotted but we failed to get its tf");
        continue;
      }

      // Calculate robot tf on global reference system multiplying the global marker tf (known a priori)
      // by marker tf on base reference system, that is, "subtract" the relative tf to the absolute one
      tf::Transform robot_gb = marker_gb*marker_bs.inverse();
      tk::tf2pose(robot_gb, pwcs->pose.pose);

      pwcs->header.stamp = msg->markers[i].header.stamp;
      pwcs->header.frame_id = global_frame_;

      robot_pose_cb_(pwcs);

      //< DEBUG
      // tf::StampedTransform tf1(marker_bs, ros::Time::now(),  base_frame_, "AR_MK");
      // tf::StampedTransform tf2(robot_gb, ros::Time::now(),  "map", "ROBOT");
      // tf_brcaster_.sendTransform(tf1);
      // tf_brcaster_.sendTransform(tf2);
      //>
    }
  }
  spotted_markers_ = *msg;

  // Decay ALL markers; that's why spotted ones got a +2 on times spotted
  for (unsigned int i = 0; i < times_spotted_.size(); i++)
  {
    if (times_spotted_[i] > 0)
      times_spotted_[i]--;
  }
}

bool ARMarkers::spotted(double younger_than,
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

bool ARMarkers::closest(const ar_track_alvar::AlvarMarkers& including,
                           const ar_track_alvar::AlvarMarkers& excluding,
                                  ar_track_alvar::AlvarMarker& closest)
{
  double closest_dist = std::numeric_limits<double>::max();
  for (unsigned int i = 0; i < spotted_markers_.markers.size(); i++)
  {
    if ((included(spotted_markers_.markers[i].id, including) == true) &&
        (excluded(spotted_markers_.markers[i].id, excluding) == true))
    {
      double d = tk::distance(spotted_markers_.markers[i].pose.pose.position);
      if (d < closest_dist)
      {
        closest_dist = d;
        closest = spotted_markers_.markers[i];
      }
    }
  }

  return (closest_dist < std::numeric_limits<double>::max());
}

bool ARMarkers::spotted(double younger_than, int min_confidence, bool exclude_globals,
                           ar_track_alvar::AlvarMarkers& spotted)
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

    if (times_spotted_[spotted_markers_.markers[i].id] >= min_confidence)
    {
      spotted.markers.push_back(spotted_markers_.markers[i]);
    }
  }

  return (spotted.markers.size() > 0);
}

bool ARMarkers::closest(double younger_than, int min_confidence, bool exclude_globals,
                           ar_track_alvar::AlvarMarker& closest)
{
  ar_track_alvar::AlvarMarkers spotted_markers;
  if (spotted(younger_than, min_confidence, exclude_globals, spotted_markers) == false)
    return false;

  double closest_dist = std::numeric_limits<double>::max();
  for (unsigned int i = 0; i < spotted_markers.markers.size(); i++)
  {
    double d = tk::distance(spotted_markers.markers[i].pose.pose.position);
    if (d < closest_dist)
    {
      closest_dist = d;
      closest = spotted_markers.markers[i];
    }
  }

  return (closest_dist < std::numeric_limits<double>::max());
}

bool ARMarkers::spotDockMarker(uint32_t base_marker_id)
{
  for (unsigned int i = 0; i < spotted_markers_.markers.size(); i++)
  {
    if (spotted_markers_.markers[i].id == base_marker_id)
    {
      if (times_spotted_[spotted_markers_.markers[i].id] < 2)
      {
        ROS_WARN("Low confidence on spotted docking marker. Dangerous...", spotted_markers_.markers[i].confidence);
      }

      docking_marker_ = spotted_markers_.markers[i];
      docking_marker_.header.frame_id = global_frame_;
      docking_marker_.pose.header.frame_id = global_frame_;

      // Get marker tf on global reference system
      tf::StampedTransform marker_gb;
      if (getMarkerTf(global_frame_, base_marker_id, docking_marker_.pose.header.stamp, marker_gb) == false)
      {
        // This should not happen unless AR tracker made a mistake, as we are using the timestamp he reported
        ROS_ERROR("Docking base marker spotted but we failed to get its tf");
        return false;
      }

      // From now we consider it as another global marker
      tk::tf2pose(marker_gb, docking_marker_.pose.pose);
      global_markers_.markers.push_back(docking_marker_);
      ROS_DEBUG("Docking AR marker registered with global pose: %.2f, %.2f, %.2f",
                docking_marker_.pose.pose.position.x, docking_marker_.pose.pose.position.y,
                tf::getYaw(docking_marker_.pose.pose.orientation));
      return true;
    }
  }

  // Cannot spot docking marker
  return false;
}

bool ARMarkers::getMarkerTf(const std::string& ref_frame, uint32_t marker_id,
                               const ros::Time& timestamp, tf::StampedTransform& tf)
{
  char marker_frame[32];
  sprintf(marker_frame, "ar_marker_%d", marker_id);

  try
  {
    // Get marker tf on given reference system
    tf_listener_.waitForTransform(ref_frame, marker_frame, timestamp, ros::Duration(0.05));
    tf_listener_.lookupTransform(ref_frame, marker_frame, timestamp, tf);

    if (tk::roll(tf) < -1.0)
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

bool ARMarkers::enableTracker()
{
  // TODO do not use by now because:
  //  - most times takes really long; I cannot find a pattern on timings
  //  - with -no kinect version of the tracker is not necessary anymore
  //  - a couple of times crashed and arduino gateway cpu usage spiked to 90%  NO IDEA WHY!!!!
  //   update: call service takes also too long; do not use
  return true;

  ros::Time t0 = ros::Time::now();
  int status = system("rosrun dynamic_reconfigure dynparam set ar_track_alvar \"{ enabled: true }\"");

  if (status != 0)
  {
    ROS_ERROR("Enable AR markers tracker failed (%d/%d)", status, WEXITSTATUS(status));
    return false;
  }
ROS_DEBUG("%f", (ros::Time::now() - t0).toSec());
  return true;
}

bool ARMarkers::disableTracker()
{
  return true;
  /*
  ros::Time t0 = ros::Time::now();
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<dynamic_reconfigure::Reconfigure>("ar_track_alvar/set_parameters");
  dynamic_reconfigure::Reconfigure srv;

  srv.request.config.doubles.resize(1);
  srv.request.config.doubles[0].name = "max_frequency";
  srv.request.config.doubles[0].value = 3.333;
  srv.request.config.bools.resize(1);
  srv.request.config.bools[0].name = "enabled";
  srv.request.config.bools[0].value = false;

  if (client.call(srv))
  {
    ROS_INFO("Sum:");
    ROS_DEBUG("%f", (ros::Time::now() - t0).toSec());
    return true;
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    ROS_DEBUG("%f", (ros::Time::now() - t0).toSec());
    return false;
  }
*/

  // TODO do not use by now because:
  //  - disable takes really long (if I use again, use instead max_frequency: 1.0 (the minimum))
  //  - with -no kinect version of the tracker is not necessary anymore
  //  - a couple of times crashed and arduino gateway cpu usage spiked to 90%  NO IDEA WHY!!!!
//  return true;

//  char system_cmd[256];
//  snprintf(system_cmd, 256,
//           "rosrun dynamic_reconfigure dynparam set ar_track_alvar \"{ enabled: true }\"");
  // TODO I think I can also call /ar_track_alvar/set_parameters... if the server is not up, system call blocks,
  // what is very shitty; another option is create a generic tk::waitForServer and reuse for action servers
  ros::Time t0 = ros::Time::now();
  int status = system("rosrun dynamic_reconfigure dynparam set ar_track_alvar \"{ enabled: false }\"");
  if (status != 0)
  {
    ROS_ERROR("Disable AR markers tracker failed (%d/%d)", status, WEXITSTATUS(status));
    return false;
  }
ROS_DEBUG("%f", (ros::Time::now() - t0).toSec());
  return true;
}


bool ARMarkers::setTrackerFreq(double freq)
{
  ros::Time t0 = ros::Time::now();
  char system_cmd[256];
  sprintf(system_cmd, "rosrun dynamic_reconfigure dynparam set ar_track_alvar \"{ max_frequency: %f }\"", freq);
  int status = system(system_cmd);
  if (status != 0)
  {
    ROS_ERROR("Set AR markers frequency tracker failed (%d/%d)", status, WEXITSTATUS(status));
    return false;
  }
ROS_DEBUG("%f", (ros::Time::now() - t0).toSec());
  return true;
}


} /* namespace waiterbot */
