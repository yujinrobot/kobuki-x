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

class IrScanNode {
public:

  /**
   * Inner class describing an individual ranger. The first four field come from
   * the rangers configuration file, and the remaining are filled on init method.
   */
  class Ranger {
  public:
    double                Q, R, P;    // KF
    unsigned int          idx_value;  /**< Index of this ranger in the msg  */
    unsigned int          idx_buffer; /**< Index of current reading in the internal buffer */
    double                last_range; /**< Internal buffer to store the latest readings    */
    int                   noisy_read;
    std::vector<int16_t>  voltage;    /**< Internal buffer to store the latest readings    */
    std::vector<double>   distance;   /**< Internal buffer to store the latest readings    */
    std::vector<double>   filt_err;   /**< Internal buffer to store the latest readings    */
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
  double max_mean_error;
  double max_covariance;
  double sensor_variance;
  double farthest_contact;
  double closest_rejected;
  int    readings_buffer;     /**< Ranger's internal buffer size */
  int    noisy_readings;
  double read_frequency;
  double ir_ring_radius;
  std::string  ir_frame_id;   /**< Frame id for the output laser scan */

  std::vector<Ranger> rangers;

  ros::Subscriber rangers_sub;
  ros::Publisher  ir_scan_pub;

  void rangersMsgCB(const arduino_resources::Rangers::ConstPtr& msg);
};

} // namespace waiterbot

#endif /* IR_SCAN_NODE_HPP_ */
