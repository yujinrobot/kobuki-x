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


namespace waiterbot
{

class VirtualSensorNode
{
public:
  VirtualSensorNode();
  ~VirtualSensorNode();

  bool init();
  void spin();

  void tablePosesCB(const semantic_region_handler::TablePoseList::ConstPtr& msg);

private:
  double         frequency_;
  double         robot_radius_;
  double         costmap_width_;
  double         costmap_height_;
  double         obstacle_height_;
  std::string     sensor_frame_id_;   /**< Frame id for the output obstacles pointcloud */
  std::string     robot_frame_id_;
  std::string     global_frame_id_;
  ros::Publisher  virtual_obs_pub_;
  ros::Subscriber table_poses_sub_;
  tf::TransformListener tf_listener_;
  std::vector<tf::Transform> obstacles_tfs_;
//  semantic_region_handler::TablePoseList table_poses_;
};

} /* namespace waiterbot */
#endif /* VIRTUAL_SENSOR_NODE_HPP_ */
