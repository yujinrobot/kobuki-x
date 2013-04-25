/*
 * common.cpp
 *
 *  Created on: Apr 11, 2013
 *      Author: jorge
 */

#include <math.h>

#include "waiterbot/common.hpp"


namespace waiterbot
{
namespace tk
{

char ___buffer___[256];
// TODO ... parameterized string composer     sprintf(marker_frame, "ar_marker_%d", msg->markers[i].id);


double wrapAngle(double a)
{
  a = fmod(a + M_PI, 2*M_PI);
  if (a < 0)
      a += 2*M_PI;
  return a - M_PI;
}

double roll(const tf::Transform& tf)
{
  double roll, pitch, yaw;
  tf::Matrix3x3(tf.getRotation()).getRPY(roll, pitch, yaw);
  return roll;
}

double roll(geometry_msgs::Pose pose)
{
  tf::Transform tf;
  pose2tf(pose, tf);
  return tk::roll(tf);
}

double roll(geometry_msgs::PoseStamped pose)
{
  return tk::roll(pose.pose);
}

double pitch(const tf::Transform& tf)
{
  double roll, pitch, yaw;
  tf::Matrix3x3(tf.getRotation()).getRPY(roll, pitch, yaw);
  return pitch;
}

double pitch(geometry_msgs::Pose pose)
{
  tf::Transform tf;
  pose2tf(pose, tf);
  return tk::pitch(tf);
}

double pitch(geometry_msgs::PoseStamped pose)
{
  return tk::pitch(pose.pose);
}

double distance(geometry_msgs::Point a, geometry_msgs::Point b)
{
  return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
}

double distance(geometry_msgs::Pose a, geometry_msgs::Pose b)
{
  return tk::distance(a.position, b.position);
}

double distance(const tf::Transform& a, const tf::Transform& b)
{
  return tf::tfDistance(a.getOrigin(), b.getOrigin());
}

double minAngle(const tf::Transform& a, const tf::Transform& b)
{
  return tf::angleShortestPath(a.getRotation(), b.getRotation());
}

void tf2pose(const tf::Transform& tf, geometry_msgs::Pose& pose)
{
  pose.position.x = tf.getOrigin().x();
  pose.position.y = tf.getOrigin().y();
  pose.position.z = tf.getOrigin().z();
  tf::quaternionTFToMsg(tf.getRotation(), pose.orientation);
}

void tf2pose(const tf::StampedTransform& tf, geometry_msgs::PoseStamped& pose)
{
  pose.header.stamp    = tf.stamp_;
  pose.header.frame_id = tf.frame_id_;
  tf2pose(tf, pose.pose);
}

void pose2tf(const geometry_msgs::Pose& pose, tf::Transform& tf)
{
  tf.setOrigin(tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
  tf::Quaternion q;
  tf::quaternionMsgToTF(pose.orientation, q);
  tf.setRotation(q);
}

void pose2tf(const geometry_msgs::PoseStamped& pose, tf::StampedTransform& tf)
{
  tf.stamp_    = pose.header.stamp;
  tf.frame_id_ = pose.header.frame_id;
  pose2tf(pose.pose, tf);
}

const char* pose2str(const geometry_msgs::Pose& pose)
{
  sprintf(___buffer___, "%.2f, %.2f, %.2f", pose.position.x, pose.position.y, tf::getYaw(pose.orientation));
  return (const char*)___buffer___;
}

const char* pose2str(const geometry_msgs::PoseStamped& pose)
{
  return pose2str(pose.pose);
}

void halfRingPoses(double radius, double height, int poses)
{
  double ang_inc = M_PI/double(poses - 1);
  for (unsigned int i = 0; i < poses; ++i)
  {
    tf::Transform r(tf::createQuaternionFromYaw(M_PI/2.0 -i*ang_inc));
    tf::Transform t(tf::createIdentityQuaternion(), tf::Vector3(radius, 0.0, height));
    tf::Transform p(r*t);
    std::cout << std::setprecision(3) << std::fixed
              << "  <node pkg=\"tf\" type=\"static_transform_publisher\" name=\"IR" << i << "_tf_publisher\"" << std::endl
              << "        args=\"" << p.getOrigin().x() << " " << p.getOrigin().y() << " " << p.getOrigin().z() << " "
              << (M_PI/2.0 -i*ang_inc) << " " << 0 << " " << 0 << " $(arg base_link) IR" << i << "_link 100\"/>" << std::endl;
  }
}


} /* namespace toolkit */
} /* namespace waiterbot */
