/*
 * ir_scan_node.cpp
 *
 *  Created on: May 31, 2012
 *      Author: jorge
 */

#include <sensor_msgs/LaserScan.h>

#include "waiterbot/common.hpp"

#include "waiterbot/ir_scan_node.hpp"

namespace waiterbot
{

IrScanNode::IrScanNode()
{
}

IrScanNode::~IrScanNode()
{
}

bool IrScanNode::read(ArduinoInterface& ai)
{
  for (unsigned int i = 0; i < rangers.size(); i++)
  {
    uint8_t data[2];
    int flags = 0; //bosch_drivers_common::SPI_READ_FLAG;

    flags = ( (0x0F & flags) | ((54 + i) << 4) ); // is this correct??
    flags = ( (0xFB & flags) | (bosch_drivers_common::MSB_FIRST << 2) );
    flags = ( (0xFC & flags) | (bosch_drivers_common::SPI_MODE_3) ); // 111111xx, where xx is the mode.

    flags = ((54 + i) << 4)  |  (bosch_drivers_common::MSB_FIRST << 2)  |  (bosch_drivers_common::SPI_MODE_3);
    //  or 86???  http://arduino.cc/en/Hacking/PinMapping2560

    ssize_t bytes_read = ai.read(0, bosch_drivers_common::SPI, 400000, &flags, 0, data, 2);

    ROS_DEBUG_STREAM(bytes_read << "    "  << flags << "    "  << i << "    "  << (int)data[1] << "   " << (int)data[0]);
//    if( hardware_->read( this->getDeviceAddress(), SPI, this->getFrequency(), this->getFlags(), ((1 << SPI_READ_FLAG)|reg), data, num_bytes ) < 0 )


  //  ROS_DEBUG_STREAM(bytes_read);

    int v = data[1];
    v <<= 8;
    v += data[0];
    double r = 1.06545479706866e-15*pow(v, 6) - 2.59219822235705e-12*pow(v, 5) + 2.52095247302813e-09*pow(v, 4)
              - 1.25091335895759e-06*pow(v, 3) + 0.000334991560873548*pow(v, 2) - 0.0469975280676629*v + 3.01895762047759;


    // KF - estimate - prediction
    rangers[i].P = rangers[i].P + rangers[i].Q;

    // KF - correction
    double z = r - rangers[i].last_range;
    double Z = rangers[i].R + rangers[i].P;

    double K = rangers[i].P/Z;

    double distanceKF = rangers[i].last_range + K*z;
    rangers[i].P = rangers[i].P - K*rangers[i].P;

    // KF - collect data
    rangers[i].last_range = distanceKF;

    if (i == 0)
    {
      // TODO This bloody sensor reports floor at 40 cm!!! until I find why....
      scan.ranges[rangers.size() - (i + 1)] = distanceKF <= 0.35 ? distanceKF + ir_ring_radius : 6.0;
//      scan.intensities[rangers.size() - (i + 1)] = distanceKF <= 0.35 ? r + ir_ring_radius : 6.0;
    }
    else
    {
      scan.ranges[rangers.size() - (i + 1)] = distanceKF <= maximum_range ? distanceKF + ir_ring_radius : 6.0;
//      scan.intensities[rangers.size() - (i + 1)] = distanceKF <= maximum_range ? r + ir_ring_radius : 6.0;
    }
    scan.intensities[rangers.size() - (i + 1)] = v;
  }
}

bool IrScanNode::spin()
{
  ArduinoInterface ai("/dev/arduino");
  if (ai.initialize() == false)
  {
    ROS_ERROR("Arduino interface initialization failed on port %s", "/dev/arduino");
    return false;
  }

  ROS_INFO("Arduino interface opened on port %s", ai.getID().c_str(), "/dev/arduino");

  scan.header.frame_id = ir_frame_id;

  ros::Rate rate(20.0);

  unsigned long long i = 0;

  while (ros::ok())
  {
    scan.header.seq = i++;
    scan.header.stamp = ros::Time::now();
    read(ai);
    ir_scan_pub.publish(scan);

    ros::spinOnce();
    rate.sleep();
  }
}

int IrScanNode::init(ros::NodeHandle& nh)
{
  // Parameters
  std::string default_frame("/ir_link");
  nh.param("ir_to_laserscan/ir_frame_id",      ir_frame_id, default_frame);
  nh.param("ir_to_laserscan/rangers_count",    rangers_count,  11);
  nh.param("ir_to_laserscan/range_variance",   range_variance,  0.025);
  nh.param("ir_to_laserscan/maximum_range",    maximum_range,   0.8);
  nh.param("ir_to_laserscan/infinity_range",   infinity_range,  6.0);
  nh.param("ir_to_laserscan/ir_ring_radius",   ir_ring_radius,  0.155);
  nh.param("ir_to_laserscan/read_frequency",   read_frequency, 20.0);

  rangers.resize(rangers_count, Ranger());

  scan.ranges.resize(rangers_count, 0.0);
  scan.intensities.resize(rangers_count, 0.0);

  scan.angle_min = -M_PI/2.0;
  scan.angle_max = +M_PI/2.0;
  scan.angle_increment = M_PI/10.0;
  scan.scan_time = 1.0 / read_frequency;
  scan.range_min = 0.10 + ir_ring_radius;
  scan.range_max = 11.00 + ir_ring_radius;

  for (unsigned int i = 0; i < rangers.size(); i++)
  {
    rangers[i].last_range = maximum_range;
    rangers[i].Q = 0.001;
    rangers[i].R = range_variance; //0.0288;
    rangers[i].P = rangers[i].R;
  }

  // Publishers and subscriptors
//  rangers_sub = nh.subscribe("rangers_data", 1, &IrScanNode::rangersMsgCB, this);
  ir_scan_pub = nh.advertise< sensor_msgs::LaserScan>("ir_scan", 1);

  ROS_INFO("IR scan node successfully initialized with %lu rangers", rangers.size());

  return 0;
}

} // namespace waiterbot

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ir_scan");

  ros::NodeHandle nh;

  waiterbot::IrScanNode node;
  if (node.init(nh) != 0)
    return -1;

  if (node.spin() == false)
    return -1;

  return 0;
}
