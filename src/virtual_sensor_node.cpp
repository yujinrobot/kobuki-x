/*
 * virtual_sensor_node.cpp
 *
 *  Created on: May 13, 2013
 *      Author: jorge
 */

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <semantic_region_handler/TablePoseList.h>

#include "waiterbot/virtual_sensor_node.hpp"


namespace waiterbot
{

VirtualSensorNode::VirtualSensorNode()
{
}

VirtualSensorNode::~VirtualSensorNode()
{
}


bool VirtualSensorNode::init()
{
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.param("obstacle_height", obstacle_height_, 0.3);
  pnh.param("sensor_frame",    sensor_frame_id_, std::string("/base_link"));
  pnh.param("robot_frame",     robot_frame_id_,  std::string("/base_footprint"));
  pnh.param("global_frame",    global_frame_id_, std::string("/map"));

  double costmap_update_frequency;
  nh.getParam("move_base/local_costmap/update_frequency", costmap_update_frequency);
  nh.getParam("move_base/local_costmap/robot_radius",     robot_radius_);
  nh.getParam("move_base/local_costmap/width",            costmap_width_);
  nh.getParam("move_base/local_costmap/height",           costmap_height_);

  frequency_ = costmap_update_frequency * 2;

  table_poses_sub_ = nh.subscribe("table_pose_list", 1, &VirtualSensorNode::tablePosesCB, this);
  virtual_obs_pub_ = nh.advertise <pcl::PointCloud<pcl::PointXYZ> > ("virtual_sensor_pointcloud", 1, true);

  return true;
}


void VirtualSensorNode::tablePosesCB(const semantic_region_handler::TablePoseList::ConstPtr& msg)
{
  // Just take first message; ignore the rest, as table list is not dynamic
  if ((obstacles_tfs_.size() == 0) && (msg->tables.size() > 0))
  {
    for (unsigned int i = 0; i < msg->tables.size(); i++)
    {
      // Skip the pickup point
      if (msg->tables[i].name.find("pickup") != std::string::npos)
        continue;

      double radius = msg->tables[i].radius;
      double theta = 0.0;
      double theta_inc = std::atan2(2.0*robot_radius_, radius);
      double tx = msg->tables[i].pose_cov_stamped.pose.pose.position.x;
      double ty = msg->tables[i].pose_cov_stamped.pose.pose.position.y;

      while (theta < 2.0*M_PI)
      {
        obstacles_tfs_.push_back(tf::Transform(tf::Quaternion::getIdentity(),
                                               tf::Vector3(tx + radius*std::cos(theta),
                                                           ty + radius*std::sin(theta),
                                                           obstacle_height_)));
        theta += theta_inc;
      }
    }
  }
}

void VirtualSensorNode::spin()
{
  ros::Rate rate(frequency_);

  while (ros::ok())
  {
    if (virtual_obs_pub_.getNumSubscribers() > 0)
    {
      pcl::PointCloud<pcl::PointXYZ> virtual_obs_;

      virtual_obs_.header.stamp = ros::Time::now();
      virtual_obs_.header.frame_id = sensor_frame_id_;

      double rx, ry;
      try
      {
        tf::StampedTransform robot_gb;
        tf_listener_.lookupTransform(global_frame_id_, robot_frame_id_, ros::Time(0.0), robot_gb);
        tf::Transform robot_gb_inv = robot_gb.inverse();

        for (unsigned int i = 0; i < obstacles_tfs_.size(); i++)
        {
          tf::Transform obs_bs = robot_gb_inv * obstacles_tfs_[i];
          if ((std::abs(obs_bs.getOrigin().x()) < costmap_width_/2.0) &&
              (std::abs(obs_bs.getOrigin().x()) < costmap_height_/2.0))
          {
            virtual_obs_.push_back(pcl::PointXYZ(obs_bs.getOrigin().x(),
                                                 obs_bs.getOrigin().y(),
                                                 obs_bs.getOrigin().z()));
          }
        }

        if (virtual_obs_.size() > 0)
          virtual_obs_pub_.publish(virtual_obs_);
      }
      catch (tf::TransformException& e)
      {
        // This cannot fail unless some part of the localization chain is missing
        ROS_WARN("Cannot get tf %s -> %s: %s", global_frame_id_.c_str(), robot_frame_id_.c_str(), e.what());
        continue;
      }



//      for (unsigned int i = 0; i < table_poses_.tables.size(); i++)
//      {
//        // Skip the pickup point
//        if (table_poses_.tables[i].name.find("pickup") != std::string::npos)
//          continue;
//
//        double radius = table_poses_.tables[i].radius;
//        double theta = 0.0;
//        double theta_inc = std::atan2(2.0*robot_radius_, radius);
//        double tx = table_poses_.tables[i].pose_cov_stamped.pose.pose.position.x;
//        double ty = table_poses_.tables[i].pose_cov_stamped.pose.pose.position.y;
//
//        // Skip obstacles out of local costmap scope
//        if ((std::abs(tx - rx) > costmap_width_/2.0) || (std::abs(ty - ry) > costmap_height_/2.0))
//          continue;
//
//        while (theta < 2.0*M_PI)
//        {
// //         if ((std::abs(tx - rx) > costmap_width_/2.0) || (std::abs(ty - ry) > costmap_height_/2.0))
//   //         continue;
//
//          virtual_obs_.push_back(pcl::PointXYZ((tx - rx) + radius*std::cos(theta),
//                                               (ty - ry) + radius*std::sin(theta),
//                                               obstacle_height_));
//          theta += theta_inc;
//        }
//      }

//      pointcloud_.resize(3);

    }

    ros::spinOnce();
    rate.sleep();
  }
}

} // namespace waiterbot


int main(int argc, char **argv)
{
  ros::init(argc, argv, "virtual_sensor");

  waiterbot::VirtualSensorNode node;
  if (node.init() == false)
  {
    ROS_ERROR("%s initialization failed", ros::this_node::getName().c_str());
    return -1;
  }
  ROS_INFO("%s initialized", ros::this_node::getName().c_str());
  node.spin();

  return 0;
}
