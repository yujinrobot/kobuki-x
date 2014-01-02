/*
  Waiter Node

  LICENSE : BSD - https://raw.github.com/yujinrobot/kobuki-x/license/LICENSE

  Author :  Jihoon Lee
  Date   : Dec 2013
 */

#ifndef _WAITER_NODE_NO_WIRELESS_HPP_
#define _WAITER_NODE_NO_WIRELESS_HPP_

#include <map>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <yocs_math_toolkit/common.hpp>

#include "waiterbot_ctrl_nowireless/navigator.hpp"
#include "waiterbot_ctrl_nowireless/tf_handlers.hpp"
#include "waiterbot_ctrl_nowireless/nav_watchdog.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <kobuki_msgs/AutoDockingAction.h>
#include <kobuki_msgs/DigitalInputEvent.h>
#include <waiterbot_msgs/NavCtrlGoTo.h>
#include <waiterbot_msgs/NavCtrlStatus.h>
#include <yocs_msgs/WaypointList.h>

#include "default_params.hpp"

namespace waiterbot {

  class WaiterIsolated {
    public: // open to everyone
      WaiterIsolated(ros::NodeHandle& n);
      ~WaiterIsolated();
      void spin();
    protected:  // internal functions
      void init();
      bool isInit();  // check if it has received waypoints 
      void digitalInputCB(const kobuki_msgs::DigitalInputEvent::ConstPtr& msg);
      void waypointsCB(const yocs_msgs::WaypointList::ConstPtr& msg);
      void commandCB(const waiterbot_msgs::NavCtrlGoTo::ConstPtr& msg);
      void trayEmptyCB(const std_msgs::Empty::ConstPtr& msg);
      void orderCancelledCB(const std_msgs::Empty::ConstPtr& msg);
      bool endCommand(const int feedback, const std::string message);
      void sendFeedback(const int feedback, const std::string message);

      bool processCommand(const int command);
        void goToVMCommand(int& feedback, std::string& message);
        void goToOriginCommand(int& feedback, std::string& message);

        bool recordOrderOrigin(std::string& message);
        bool goToVendingMachine(std::string& message);
        bool goToOrigin(std::string& message);

      bool dockInBase();

      void playSound(const std::string& wav_file);
    private: // variables
      ros::NodeHandle nh_;
      ros::Publisher  pub_navctrl_feedback_;
      ros::Subscriber sub_digital_input_;
      ros::Subscriber sub_waypoints_;
      ros::Subscriber sub_navctrl_;
      ros::Subscriber sub_order_cancelled_;
      ros::Subscriber sub_tray_empty_;
      actionlib::SimpleActionClient<kobuki_msgs::AutoDockingAction> ac_autodock_;

      std::string resources_path_;
      std::string loc_vm_;
      std::string loc_customer_;
      std::string base_frame_;
      std::string odom_frame_;
      std::string global_frame_;

      bool initialized_;
      bool waypointsReceived_;
      bool inCommand_;

      Navigator navigator_;
      NavWatchdog watchdog_;
      TFHandlers tf_handlers_;

      std::map<std::string, geometry_msgs::PoseStamped> map_wp_;
      geometry_msgs::PoseStamped robot_origin_;

      bool digital_input_first_time_;
      kobuki_msgs::DigitalInputEvent prev_digital_input;

      boost::thread command_process_thread_;

      bool cancel_order_;
      bool tray_empty_;
      bool in_docking_;
  };
}

#endif // _WAITER_NODE_NO_WIRELESS_HPP_
