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
   * Inner class describing an individual sonar. The first four field come from
   * the sonars configuration file, and the remaining are filled on init method.
   */
  class Sonar {
  public:
    double Q, R, P;  // KF

    unsigned int id;
    std::string            name;       /**< Sonar's name (normally its place in the robot)  */
    std::string            frame_id;   /**< Reference frame id; needed to lookup transform  */
    unsigned int           idx_fired;  /**< Index that identifies when this sonar is fired  */
    unsigned int           idx_value;  /**< Index of this sonar in the msg first echo data  */
    unsigned int           idx_buffer; /**< Index of current reading in the internal buffer */
    tf::Transform          transform;  /**< Frame according to which we transform reading   */
    double                 last_range; /**< Internal buffer to store the latest readings    */
    int                    noisy_read;
    std::vector<int16_t>   voltage;    /**< Internal buffer to store the latest readings    */
    std::vector<double>    distance;   /**< Internal buffer to store the latest readings    */
    std::vector<double>    filt_err;   /**< Internal buffer to store the latest readings    */
    std::vector<ros::Time> t_stamps;   /**< Internal buffer to store the latest timestamps  */
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
  double farthest_contact;
  double closest_rejected;
  int    readings_buffer;
  int    noisy_readings;
  double frequency;
  unsigned int sonar_buffer_size; /**< Sonar's internal buffer size */
  unsigned int sonar_max_reading; /**< Sonar's max reported distance */
  unsigned int no_contact_range;  /**< Report this distance if no contact */

  std::vector<Sonar> sonars;

  std::string pcloud_frame;    /**< Frame id for the output point cloud */
  std::string sonars_cfg_file; /**< Sonars configuration YAML file path */

  ros::Subscriber sonars_sub;
  ros::Publisher  pcloud_pub;

//  bool loadSonarsCfg(std::string path);
  void sonarsMsgCB(const arduino_resources::Rangers::ConstPtr& msg);

  uint8_t median(std::vector<uint8_t> values) {
    // Return the median element of an integers vector
    nth_element(values.begin(), values.begin() + values.size()/2, values.end());
    return values[values.size()/2];
  }
};

} // namespace waiterbot

#endif /* IR_SCAN_NODE_HPP_ */
