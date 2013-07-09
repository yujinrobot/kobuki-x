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

#include <math_toolkit/geometry.hpp>

#include <semantic_region_handler/TablePoseList.h>

#include "virtual_sensor/WallList.h"
#include "virtual_sensor/ColumnList.h"

namespace virtual_sensor
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

  class Column : public Obstacle
  {
  public:
    Column(const tf::Transform& tf, double radius, double height)
    {
      tf_     = tf;
      radius_ = radius;
      height_ = height;

      distance_ = mtk::distance2D(tf.getOrigin()) - radius;
    }

    double radius() { return radius_; }
    bool intersects(double rx, double ry, double max_dist, double& distance)
    {
      double ix, iy;
      bool intersects =
          mtk::rayCircleIntersection(rx, ry, tf_.getOrigin().x(), tf_.getOrigin().y(), radius_, ix, iy, distance);
      if ((intersects == false) || (distance > max_dist))
        return false;
      else
        return true;
    }


  private:
    double radius_;
    double height_;
  };

  class Wall : public Obstacle
  {
  public:
    Wall(const tf::Transform& tf, double length, double width, double height)
    {
      tf_     = tf;
      length_ = length;
      height_ = height;
      width_  = width;

      p1_.setX(- width_/2.0);
      p1_.setY(- length_/2.0);
      p1_.setZ(  0.0);
      p2_.setX(+ width_/2.0);
      p2_.setY(+ length_/2.0);
      p2_.setZ(+ height_);

      p1_ = tf_ * p1_;
      p2_ = tf_ * p2_;

      distance_ = mtk::pointSegmentDistance(0.0, 0.0, p1_.x(), p1_.y(), p2_.x(), p2_.y());
    }

    double length() { return length_; }
    double height() { return height_; }
    double width()  { return width_;  }
    bool intersects(double rx, double ry, double max_dist, double& distance)
    {
      double ix, iy;
      bool intersects =
          mtk::raySegmentIntersection(0.0, 0.0, rx, ry, p1_.x(), p1_.y(), p2_.x(), p2_.y(), ix, iy, distance);
      if ((intersects == false) || (distance > max_dist))
        return false;
      else
        return true;
    }

  private:
    tf::Point p1_;
    tf::Point p2_;
    double length_;
    double height_;
    double width_;
  };

  VirtualSensorNode();
  ~VirtualSensorNode();

  bool init();
  void spin();

  void tablePosesCB(const semantic_region_handler::TablePoseList::ConstPtr& msg);
  void columnPosesCB(const virtual_sensor::ColumnList::ConstPtr& msg);
  void wallPosesCB(const virtual_sensor::WallList::ConstPtr& msg);

private:
  double         angle_min_;
  double         angle_max_;
  double         angle_inc_;
  double         scan_time_;
  double         range_min_;
  double         range_max_;
  double         frequency_;
  int             hits_count_;
  std::string     sensor_frame_id_;   /**< Frame id for the output scan */
  std::string     global_frame_id_;
  ros::Publisher  virtual_obs_pub_;
  ros::Subscriber wall_poses_sub_;
  ros::Subscriber column_poses_sub_;
  ros::Subscriber table_poses_sub_;
  sensor_msgs::LaserScan     scan_;
  tf::TransformListener tf_listener_;
  std::vector<semantic_region_handler::TablePose> circles_;
  std::vector<virtual_sensor::Column> columns_;
  std::vector<virtual_sensor::Wall>   walls_;

  /**
   * Add a new obstacle to obstacles list with some processing:
   *  - remove those out of range
   *  - short by increasing distance to the robot
   * @param new_obs   New obstacle in robot reference system
   * @param obstacles Current obstacles list
   * @return True if added, false otherwise
   */
  bool add(boost::shared_ptr<Obstacle>& new_obs, std::vector< boost::shared_ptr<Obstacle> >& obstacles);
};

} /* namespace virtual_sensor */
#endif /* VIRTUAL_SENSOR_NODE_HPP_ */
