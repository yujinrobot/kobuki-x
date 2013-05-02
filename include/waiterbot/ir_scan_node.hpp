/*
 * ir_scan_node.hpp
 *
 *  Created on: Apr 17, 2013
 *      Author: jorge
 */

#ifndef IR_SCAN_NODE_HPP_
#define IR_SCAN_NODE_HPP_


#include <ros/ros.h>
#include <arduino_resources/Rangers.h>

namespace waiterbot
{

class IrScanNode
{
public:

  /**
   * Inner class describing an individual ranger
   */
  class Ranger
  {
  public:
    double Q, R, P;    // KF
    double last_range; /**< Internal buffer to store the latest readings    */
  };

  /*********************
  ** Initialization
  **********************/
  IrScanNode();
  ~IrScanNode();
  int init(ros::NodeHandle& nh);

  /*********************
  ** Runtime
  **********************/
  int spin();

private:
  int      rangers_count;
  double   range_variance;
  double   maximum_range;
  double   infinity_range;
  double   read_frequency;
  double   ir_ring_radius;
  std::string  ir_frame_id;   /**< Frame id for the output laser scan */

  sensor_msgs::LaserScan scan;

  std::vector<Ranger> rangers;

  ros::Subscriber rangers_sub;
  ros::Publisher  ir_scan_pub;

  void rangersMsgCB(const arduino_resources::Rangers::ConstPtr& msg);
};

} // namespace waiterbot

#endif /* IR_SCAN_NODE_HPP_ */
