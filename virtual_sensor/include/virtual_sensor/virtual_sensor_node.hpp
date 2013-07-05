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

    double dist2d(double x, double y)
    {
      return std::sqrt(std::pow(x, 2) + std::pow(y, 2));
    }

    double dist2d(const tf::Point& p)
    {
      return std::sqrt(std::pow(p.x(), 2) + std::pow(p.y(), 2));
    }

    double dist3d(const tf::Point& p)
    {
      return std::sqrt(std::pow(p.x(), 2) + std::pow(p.y(), 2) + std::pow(p.z(), 2));
    }

    double dist2d(const tf::Point& p1, const tf::Point& p2)
    {
      return std::sqrt(std::pow(p2.x() - p1.x(), 2) + std::pow(p2.y() - p1.y(), 2));
    }

    double dist3d(const tf::Point& p1, const tf::Point& p2)
    {
      return std::sqrt(std::pow(p2.x() - p1.x(), 2) + std::pow(p2.y() - p1.y(), 2) + std::pow(p2.z() - p1.z(), 2));
    }
  };

  class Circle : public Obstacle
  {
  public:
    Circle(const tf::Transform& tf, double radius)
    {
      tf_     = tf;
      radius_ = radius;

      distance_ = std::sqrt(std::pow(tf.getOrigin().x(), 2) + std::pow(tf.getOrigin().y(), 2)) - radius;
    }

    double radius() { return radius_; }
    bool intersects(double rx, double ry, double max_dist, double& distance);

  private:
    double radius_;
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

      distance_ = dist(p1_, p2_);
    }

    double dist(const tf::Point& p1, const tf::Point& p2)
    {
      // Return minimum distance between line segment vw and point p
//
//
//        const float l2 = length_squared(v, w);  // i.e. |w-v|^2 -  avoid a sqrt
//        if (l2 == 0.0) return distance(p, v);   // v == w case
//        // Consider the line extending the segment, parameterized as v + t (w - v).
//        // We find projection of point p onto the line.
//        // It falls where t = [(p-v) . (w-v)] / |w-v|^2
//        const float t = dot(p - v, w - v) / l2;
//        if (t < 0.0) return distance(p, v);       // Beyond the 'v' end of the segment
//        else if (t > 1.0) return distance(p, w);  // Beyond the 'w' end of the segment
//        const vec2 projection = v + t * (w - v);  // Projection falls on the segment
//        return distance(p, projection);


          double l = dist2d(p1, p2);
          if (l == 0.0) return dist2d(p1);
          double t = (- p1.x() * (p2.x() - p1.x()) - p1.y() * (p2.y() - p1.y())) / l;
          if (t < 0.0) return dist2d(p1);
          if (t > 1.0) return dist2d(p2);
          return dist2d(p1.x() + t * (p2.x() - p1.x()), p1.y() + t * (p2.y() - p1.y()));
    }

    double length() { return length_; }
    double height() { return height_; }
    double width()  { return width_;  }
    bool intersects(double rx, double ry, double max_dist, double& distance);

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
