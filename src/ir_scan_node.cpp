/*
 * ir_scan_node.cpp
 *
 *  Created on: May 31, 2012
 *      Author: jorge
 */

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/LaserScan.h>

//#include <yaml-cpp/yaml.h>

#include "waiterbot/common.hpp"

#include "waiterbot/ir_scan_node.hpp"

namespace waiterbot
{

std::vector<double> kkk;
IrScanNode::IrScanNode()
{
}

IrScanNode::~IrScanNode()
{
}

void IrScanNode::rangersMsgCB(const arduino_resources::Rangers::ConstPtr& msg)
{
  sensor_msgs::LaserScan scan;
  scan.header = msg->header;
  scan.header.frame_id = ir_frame_id;
  scan.ranges.resize(msg->ranges.size(), 0.0);
  scan.intensities.resize(msg->ranges.size(), 0.0);
//std algo   scan.intensities = msg->ranges;

  scan.angle_min = -M_PI/2.0;
  scan.angle_max = +M_PI/2.0;
  scan.angle_increment = M_PI/10.0;
//  scan.time_increment
  scan.scan_time = 1.0 / read_frequency;
  scan.range_min = 0.10 + ir_ring_radius;
  scan.range_max = 11.00 + ir_ring_radius;

  for (unsigned int i = 0; i < msg->ranges.size(); i++)
  {
    double v = msg->ranges[i]; // no filter by now
    double r = 1.06545479706866e-15*pow(v, 6) - 2.59219822235705e-12*pow(v, 5) + 2.52095247302813e-09*pow(v, 4)
             - 1.25091335895759e-06*pow(v, 3) + 0.000334991560873548*pow(v, 2) - 0.0469975280676629*v + 3.01895762047759;

    // identify the position of its reading in the incoming message data
    int idx = rangers[i].idx_buffer;
    // store reading a timestamp in the proper place of its internal buffer
//    rangers[i].readings[idx] = r;
////    rangers[i].t_stamps[idx] = msg->header.stamp;
//    // and increment the internal buffer index
//    rangers[i].idx_buffer  = (rangers[i].idx_buffer + 1) % readings_buffer;


//    FILE* f = fopen("COV.m", "w");
//fprintf(file, "%f\t", r);
//    double r = 7.17062444495719e-15*pow(v, 6) - 1.23976199902831e-11*pow(v, 5) + 8.66526083489264e-09*pow(v, 4)
//             - 3.15513738809603e-06*pow(v, 3) + 0.000639205288260282*pow(v, 2) - 0.0704748189962500*v + 3.69769525786646;

//    prevDist = 0;
//
    // estimate - prediction
    rangers[i].P = rangers[i].P + rangers[i].Q;

    // correction
   // double E = H * P * H';

    double z = r - rangers[i].last_range;
    double Z = rangers[i].R + rangers[i].P;

    double K = rangers[i].P/Z;

    double distanceKF = rangers[i].last_range + K*z;
    rangers[i].P = rangers[i].P - K*rangers[i].P;

    // collect data
    rangers[i].last_range = distanceKF;

    rangers[i].voltage[idx]  = v;
    rangers[i].distance[idx] = r;
    rangers[i].filt_err[idx] = distanceKF - r;

//    rangers[i].t_stamps[idx] = msg->header.stamp;
    // and increment the internal buffer index
    rangers[i].idx_buffer  = (rangers[i].idx_buffer + 1) % readings_buffer;

//    int idx1 = rangers[i].idx_buffer;
//    int idx2 = (idx + 1) % readings_buffer;
    //idx--;
    //if (idx < 0) idx =  readings_buffer-1;
    //    return distanceKF;

    // inverse order, as for laser scan first value correspond to A10
    //scan.intensities[msg->ranges.size() - (i + 1)] = r + ir_ring_radius;//(r <= 110.6)?(r + ir_ring_radius):6.0;

   // double var = tk::variance(rangers[i].distance);

    // Filter by checking accumulated difference between filtered and unfiltered ranges
    double acc_filt_err = 0.0;
    for (unsigned int j = 0; j < readings_buffer; j++)
    {
      acc_filt_err += std::abs(rangers[i].filt_err[j]);
    }
    double avg_filt_err = acc_filt_err/readings_buffer;

    // Filter by checking accumulated difference between consecutive raw readings
    double acc_abs_diff = 0.0;
    double acc_sgn_diff = 0.0;

    for (unsigned int j = 1; j < readings_buffer; j++)
    {
      int idx1 = (idx  + j) % readings_buffer;
      int idx2 = (idx1 + 1) % readings_buffer;

      double diff = rangers[i].voltage[idx2] - rangers[i].voltage[idx1];
      acc_abs_diff += std::abs(diff);
      acc_sgn_diff +=          diff;
    }

    double avg_abs_diff = acc_abs_diff/readings_buffer - 1;
    double avg_sgn_diff = acc_sgn_diff/readings_buffer - 1;

 //   fprintf(f, "  %f  %f  %f  %f\n", avg_abs_diff, avg_sgn_diff, avg_sgn_diff/avg_abs_diff, tk::mean(rangers[i].distance));

    // Filter by adaptive variance threshold
    double var_threshold = max_covariance*(0.5 + std::pow(std::max(0.0, std::min(1.0, (distanceKF - closest_rejected)/(farthest_contact - closest_rejected))), 2));

    if (tk::variance(rangers[i].distance) > var_threshold)
      rangers[i].noisy_read = std::min(rangers[i].noisy_read + 1, + noisy_readings);
    else
      rangers[i].noisy_read = std::max(rangers[i].noisy_read - 1, - noisy_readings);

   // scan.ranges[msg->ranges.size() - (i + 1)] = ((distanceKF <= farthest_contact) && (rangers[i].noisy_read <= 0.0)) || (distanceKF < closest_rejected)?(distanceKF + ir_ring_radius):6.0;
    scan.ranges[msg->ranges.size() - (i + 1)] = distanceKF <= farthest_contact ? distanceKF + ir_ring_radius : 6.0;
    scan.intensities[msg->ranges.size() - (i + 1)] = distanceKF <= farthest_contact ? r + ir_ring_radius : 6.0;;//avg_abs_diff;//tk::variance(rangers[i].distance);//r + ir_ring_radius;//(r <= 110.6)?(r + ir_ring_radius):6.0;


    if (i == 55555)
    {
      if (avg_abs_diff >= max_mean_error)
        scan.intensities[0] = 0.55;
      else
        scan.intensities[0] = 0.0;

      if (avg_filt_err >= max_mean_error/100.0)
        scan.intensities[1] = 0.5;
      else
        scan.intensities[1] = 0.0;

      if (tk::variance(rangers[i].distance) >= max_covariance)
        scan.intensities[2] = 0.45;
      else
        scan.intensities[2] = 0.0;

      if (tk::variance(rangers[i].voltage) >= max_mean_error*10.0)
        scan.intensities[3] = 0.4;
      else
        scan.intensities[3] = 0.0;

      double var_threshold = max_covariance*(0.5 + std::pow(std::max(0.0, std::min(1.0, (distanceKF - closest_rejected)/(farthest_contact - closest_rejected))), 2));

      if (tk::variance(rangers[i].distance) >=  var_threshold)
          scan.intensities[4] = 0.3;
      else
        scan.intensities[4] = 0.0;

      scan.intensities[msg->ranges.size() - (i + 1)] = r + ir_ring_radius;//avg_abs_diff;//tk::variance(rangers[i].distance);//r + ir_ring_radius;//(r <= 110.6)?(r + ir_ring_radius):6.0;

      scan.intensities[6] = var_threshold;
      scan.intensities[7] = tk::variance(rangers[i].distance);
      scan.intensities[8] = rangers[i].noisy_read;

//      fprintf(f, "%f %f %f %f %f %f %f %f %d %d\n", v, r, distanceKF, tk::variance(rangers[i].distance), tk::std_dev(rangers[i].distance), avg_filt_err, avg_abs_diff, avg_sgn_diff,
//              avg_filt_err >= max_covariance/100.0, avg_abs_diff >= max_covariance);
    }
  }

//  if (rangers[0].idx_buffer == 0)
//  {
//    for (unsigned int i = 0; i < msg->ranges.size(); i++) {
//
//      double mean = 0;
//      for (unsigned int j = 0; j < readings_buffer; j++)
//        mean += rangers[i].readings[j];
//
//      double v = mean/readings_buffer;//     msg->ranges[i]; // no filter by now
//      double r = 1.06545479706866e-15*pow(v, 6) - 2.59219822235705e-12*pow(v, 5) + 2.52095247302813e-09*pow(v, 4)
//               - 1.25091335895759e-06*pow(v, 3) + 0.000334991560873548*pow(v, 2) - 0.0469975280676629*v + 3.01895762047759;
//
//      scan.ranges[i] = r;
//      scan.intensities[i] = v;
//    }
  ir_scan_pub.publish(scan);
//  }
}

int IrScanNode::init(ros::NodeHandle& nh)
{
  // Parameters
  std::string default_frame("ir_link");
  nh.param("ir_to_laserscan/ir_frame_id",      ir_frame_id, default_frame);
  nh.param("ir_to_laserscan/max_mean_error",   max_mean_error,   0.01);
  nh.param("ir_to_laserscan/max_covariance",   max_covariance,   0.01);
  nh.param("ir_to_laserscan/sensor_variance",  sensor_variance,  0.025);
  nh.param("ir_to_laserscan/readings_buffer",  readings_buffer,  5);
  nh.param("ir_to_laserscan/farthest_contact", farthest_contact, 0.8);
  nh.param("ir_to_laserscan/closest_rejected", closest_rejected, 0.4);
  nh.param("ir_to_laserscan/noisy_readings",   noisy_readings,  10);
  nh.param("ir_to_laserscan/ir_ring_radius",   ir_ring_radius,   0.155);
  nh.param("ir_to_laserscan/read_frequency",   read_frequency,  20.0);

  rangers.resize(11, Ranger());
  const ros::Time       t0(0);
  const ros::Duration   d4(4);
  tf::StampedTransform  stf;
  tf::TransformListener tfl;

  for (unsigned int i = 0; i < rangers.size(); i++)
  {
    // Allocate ranger readings and other internal buffers
    rangers[i].idx_buffer = 0;
    rangers[i].noisy_read = 0;
    rangers[i].last_range = farthest_contact;
    rangers[i].voltage.resize(readings_buffer, 0);
    rangers[i].distance.resize(readings_buffer, 0);
    rangers[i].filt_err.resize(readings_buffer, 0);


    rangers[i].Q = 0.001;
    rangers[i].R = sensor_variance; //0.0288;
    rangers[i].P = rangers[i].R;
  }

  // Publishers and subscriptors
  rangers_sub = nh.subscribe("rangers_data", 1, &IrScanNode::rangersMsgCB, this);
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

  ros::spin();

  return 0;
}
