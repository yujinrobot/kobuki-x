/*
 * common.hpp
 *
 *  Created on: Apr 11, 2013
 *      Author: jorge
 */

#ifndef COMMON_HPP_
#define COMMON_HPP_


#include <tf/tf.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>


namespace waiterbot
{
namespace tk // toolkit
{

double distance(geometry_msgs::Point a, geometry_msgs::Point b = geometry_msgs::Point());

double distance(geometry_msgs::Pose a, geometry_msgs::Pose b = geometry_msgs::Pose());

void tf2pose(const tf::Transform& tf, geometry_msgs::Pose& pose);

void tf2pose(const tf::StampedTransform& tf, geometry_msgs::PoseStamped& pose);

void pose2tf(const geometry_msgs::Pose& pose, tf::Transform& tf);

void pose2tf(const geometry_msgs::PoseStamped& pose, tf::StampedTransform& tf);

} /* namespace toolkit */
} /* namespace waiterbot */

#endif /* COMMON_HPP_ */
