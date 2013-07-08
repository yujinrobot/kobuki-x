/*
 * geometry.cpp
 *
 *  Created on: Apr 11, 2013
 *      Author: jorge
 */

#include <math.h>

#include "math_toolkit/common.hpp"
#include "math_toolkit/geometry.hpp"


namespace mtk
{


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
  return roll(tf);
}

double roll(geometry_msgs::PoseStamped pose)
{
  return roll(pose.pose);
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
  return pitch(tf);
}

double pitch(geometry_msgs::PoseStamped pose)
{
  return pitch(pose.pose);
}

double distance3D(const tf::Vector3& a, const tf::Vector3& b)
{
  return tf::tfDistance(a, b);
}

double distance3D(geometry_msgs::Point a, geometry_msgs::Point b)
{
  return distance3D(tf::Vector3(a.x, a.y, a.z), tf::Vector3(b.x, b.y, b.z));
}

double distance3D(geometry_msgs::Pose a, geometry_msgs::Pose b)
{
  return distance3D(a.position, b.position);
}

double distance3D(const tf::Transform& a, const tf::Transform& b)
{
  return distance3D(a.getOrigin(), b.getOrigin());
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
  return distance2D(tf::Vector3(a.x, a.y, a.z), tf::Vector3(b.x, b.y, b.z));
}

double distance2D(geometry_msgs::Pose a, geometry_msgs::Pose b)
{
  return distance2D(a.position, b.position);
}

double distance2D(const tf::Transform& a, const tf::Transform& b)
{
  return distance2D(a.getOrigin(), b.getOrigin());
}

double heading(const tf::Vector3& a, const tf::Vector3& b)
{
  return std::atan2(b.y() - a.y(), b.x() - a.x());
}

double heading(geometry_msgs::Point a, geometry_msgs::Point b)
{
  return heading(tf::Vector3(a.x, a.y, a.z), tf::Vector3(b.x, b.y, b.z));
}

double heading(geometry_msgs::Pose a, geometry_msgs::Pose b)
{
  return heading(a.position, b.position);
}

double heading(const tf::Transform& a, const tf::Transform& b)
{
  return heading(a.getOrigin(), b.getOrigin());
}

double minAngle(const tf::Quaternion& a, const tf::Quaternion& b)
{
  return tf::angleShortestPath(a, b);
}

double minAngle(geometry_msgs::Quaternion a, geometry_msgs::Quaternion b)
{
  return minAngle(tf::Quaternion(a.x, a.y, a.z, a.w), tf::Quaternion(b.x, b.y, b.z, b.w));
}

double minAngle(geometry_msgs::Pose a, geometry_msgs::Pose b)
{
  return minAngle(a.orientation, b.orientation);
}

double minAngle(const tf::Transform& a, const tf::Transform& b)
{
  return minAngle(a.getRotation(), b.getRotation());
}


} /* namespace mtk */
