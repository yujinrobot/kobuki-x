/*
  TF Handlers 

  LICENSE : BSD - https://raw.github.com/yujinrobot/kobuki-x/license/LICENSE

  Author :  Jihoon Lee
  Date   : Dec 2013
 */

#ifndef _WAITER_NODE_TF_HANDLERS_HPP_
#define _WAITER_NODE_TF_HANDLERS_HPP_

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

namespace waiterbot {
  class TFHandlers {
    public:
      bool getTf(const std::string& frame_1, const std::string& frame_2,tf::StampedTransform& tf);
      
    private:
      tf::TransformListener tf_listener_;
      tf::TransformBroadcaster tf_brcaster_;
  };
}

#endif // _WAITER_NODE_TF_HANDLERS_HPP_
