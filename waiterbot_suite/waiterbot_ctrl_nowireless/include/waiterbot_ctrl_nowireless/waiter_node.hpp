/*
  Waiter Node

  LICENSE : BSD - https://raw.github.com/yujinrobot/kobuki-x/license/LICENSE# is used, also find other catkin packages

  Author :  Jihoon Lee
  Date   : Dec 2013
 */

#ifndef _WAITER_NODE_NO_WIRELESS_HPP_
#define _WAITER_NODE_NO_WIRELESS_HPP_

#include <ros/ros.h>

namespace waiterbot {
  class WaiterIsolated {
    public: // open to everyone
      WaiterIsolated();
      ~WaiterIsolated();
      void spin();
    protected:  // internal functions
    private: // variables
  };
}

#endif // _WAITER_NODE_NO_WIRELESS_HPP_
