/*
 * virtual_sensor_node.hpp
 *
 *  Created on: May 13, 2013
 *      Author: jorge
 */

#ifndef VIRTUAL_SENSOR_NODE_HPP_
#define VIRTUAL_SENSOR_NODE_HPP_

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>

#include <semantic_region_handler/TablePoseList.h>

namespace waiterbot
{

class VirtualSensorNode
{
public:

  class Obstacle
  {
  public:
    double distance() { return distance_; }
    virtual bool intersects(double rx, double ry, double max_dist, double& distance) = 0;

  protected:
    tf::Transform tf_;
    double distance_;
  };

  class Circle : public Obstacle
  {
  public:
    Circle(const tf::Transform& tf, double radius)
    {
      tf_ = tf;
      radius_ = radius;

      distance_ = std::sqrt(std::pow(tf.getOrigin().x(), 2) + std::pow(tf.getOrigin().y(), 2)) - radius;
    }

    double radius() { return radius_; }
    bool intersects(double rx, double ry, double max_dist, double& distance);

  private:
    double radius_;
  };

  VirtualSensorNode();
  ~VirtualSensorNode();

  bool init();
  void spin();

  void tablePosesCB(const semantic_region_handler::TablePoseList::ConstPtr& msg);

private:
  double         angle_min_;
  double         angle_max_;
  double         angle_inc_;
  double         scan_time_;
  double         range_min_;
  double         range_max_;
  double         frequency_;
  std::string     sensor_frame_id_;   /**< Frame id for the output scan */
  std::string     global_frame_id_;
  ros::Publisher  virtual_obs_pub_;
  ros::Subscriber table_poses_sub_;
  sensor_msgs::LaserScan     scan_;
  tf::TransformListener tf_listener_;
  std::vector<tf::Transform> obstacles_tfs_;
  std::vector<semantic_region_handler::TablePose> circles_;
};

} /* namespace waiterbot */
#endif /* VIRTUAL_SENSOR_NODE_HPP_ */
