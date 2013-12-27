/*
  TfHandlers Class

  LICENSE : BSD - https://raw.github.com/yujinrobot/kobuki-x/license/LICENSE

  Author :  Jihoon Lee
  Date   : Dec 2013
 */


#include "waiterbot_ctrl_nowireless/tf_handlers.hpp"

namespace waiterbot {

  bool TFHandlers::getTf(const std::string& frame_1, const std::string& frame_2,tf::StampedTransform& tf)
  {
    // Use this just to get tf that cannot fail unless some part of the localization chain is missing
    // Otherwise said; if this exception happens we are really pissed-off, so we don't try to recover
    try
    {
      tf_listener_.lookupTransform(frame_1, frame_2, ros::Time(0.0), tf);
    }
    catch (tf::TransformException& e)
    {
      ROS_WARN("Cannot get tf %s -> %s: %s", frame_1.c_str(), frame_1.c_str(), e.what());
      return false;
    }
    return true;
  }
}

