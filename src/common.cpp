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

double sign(double x)
{
  return x > 0.0 ? +1.0 : x < 0.0 ? -1.0 : 0.0;
}

double wrapAngle(double a)
{
  a = fmod(a + M_PI, 2*M_PI);
  if (a < 0.0)
      a += 2.0*M_PI;
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

double distance3D(const tf::Vector3& a, const tf::Vector3& b)
{
  return tf::tfDistance(a, b);
}

double distance3D(geometry_msgs::Point a, geometry_msgs::Point b)
{
  return tk::distance3D(tf::Vector3(a.x, a.y, a.z), tf::Vector3(b.x, b.y, b.z));
}

double distance3D(geometry_msgs::Pose a, geometry_msgs::Pose b)
{
  return tk::distance3D(a.position, b.position);
}

double distance3D(const tf::Transform& a, const tf::Transform& b)
{
  return tk::distance3D(a.getOrigin(), b.getOrigin());
}

double distance2D(double ax, double ay, double bx, double by)
{
  return std::sqrt(std::pow(ax - bx, 2) + std::pow(ay - by, 2));
}

double distance2D(const tf::Vector3& a, const tf::Vector3& b)
{
  return std::sqrt(std::pow(a.x() - b.x(), 2) + std::pow(a.y() - b.y(), 2));
}

double distance2D(geometry_msgs::Point a, geometry_msgs::Point b)
{
  return tk::distance2D(tf::Vector3(a.x, a.y, a.z), tf::Vector3(b.x, b.y, b.z));
}

double distance2D(geometry_msgs::Pose a, geometry_msgs::Pose b)
{
  return tk::distance2D(a.position, b.position);
}

double distance2D(const tf::Transform& a, const tf::Transform& b)
{
  return tk::distance2D(a.getOrigin(), b.getOrigin());
}

double heading(const tf::Vector3& a, const tf::Vector3& b)
{
  return std::atan2(b.y() - a.y(), b.x() - a.x());
}

double heading(geometry_msgs::Point a, geometry_msgs::Point b)
{
  return tk::heading(tf::Vector3(a.x, a.y, a.z), tf::Vector3(b.x, b.y, b.z));
}

double heading(geometry_msgs::Pose a, geometry_msgs::Pose b)
{
  return tk::heading(a.position, b.position);
}

double heading(const tf::Transform& a, const tf::Transform& b)
{
  return tk::heading(a.getOrigin(), b.getOrigin());
}

double minAngle(const tf::Quaternion& a, const tf::Quaternion& b)
{
  return tf::angleShortestPath(a, b);
}

double minAngle(geometry_msgs::Quaternion a, geometry_msgs::Quaternion b)
{
  return tk::minAngle(tf::Quaternion(a.x, a.y, a.z, a.w), tf::Quaternion(b.x, b.y, b.z, b.w));
}

double minAngle(geometry_msgs::Pose a, geometry_msgs::Pose b)
{
  return tk::minAngle(a.orientation, b.orientation);
}

double minAngle(const tf::Transform& a, const tf::Transform& b)
{
  return tk::minAngle(a.getRotation(), b.getRotation());
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
