/*
 * ir_scan_node.hpp
 *
 *  Created on: Apr 17, 2013
 *      Author: jorge
 */

#ifndef IR_SCAN_NODE_HPP_
#define IR_SCAN_NODE_HPP_


#include <ros/ros.h>
#include <arduino_interface.hpp>
#include <adc_driver/adc_driver.h>

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
    boost::shared_ptr<AdcDriver> adc_driver;
  };

  /*********************
  ** Initialization
  **********************/
  IrScanNode();
  ~IrScanNode();
  bool spin();
  bool init(ros::NodeHandle& nh);

private:
  int      rangers_count;
  double   range_variance;
  double   maximum_range;
  double   infinity_range;
  double   read_frequency;
  double   ir_ring_radius;
  std::string ir_frame_id;   /**< Frame id for the output laser scan */
  std::string arduino_port;

  sensor_msgs::LaserScan scan;

  std::vector<Ranger> rangers;

  ros::Publisher  ir_scan_pub;

  boost::shared_ptr<ArduinoInterface> arduino_iface;

  bool readRanges();
};

} // namespace waiterbot

#endif /* IR_SCAN_NODE_HPP_ */
