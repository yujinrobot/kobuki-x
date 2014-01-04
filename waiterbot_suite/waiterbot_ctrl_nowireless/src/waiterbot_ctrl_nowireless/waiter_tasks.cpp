/*
  WaiterIsolated Class

  LICENSE : BSD - https://raw.github.com/yujinrobot/kobuki-x/license/LICENSE

  Author :  Jihoon Lee
  Date   : Dec 2013
 */

#include "waiterbot_ctrl_nowireless/waiter_node.hpp"

namespace waiterbot {
bool WaiterIsolated::recordOrderOrigin(std::string& message)
{
  // base frame is robot's pose on global frame
  tf::StampedTransform robot_tf;
  
  if(tf_handlers_.getTf(global_frame_, base_frame_,robot_tf) == false) 
  {
    std::stringstream sstm;
    sstm << "Failed to get transform between " << global_frame_ << " and " << base_frame_;
    message = sstm.str();
    return false;
  }

  mtk::tf2pose(robot_tf, robot_origin_);
  ROS_INFO("Waiter : Order received from %.2f %.2f %.2f", robot_origin_.pose.position.x, robot_origin_.pose.position.y, tf::getYaw(robot_origin_.pose.orientation));

  return true;
}

bool WaiterIsolated::goToVendingMachine(std::string& message)
{
  // go to in front of vending machine
  geometry_msgs::PoseStamped vm;// = map_wp_[loc_vm_];
  tf::StampedTransform vm_tf;
  // get target pose frame transform tree
  if(tf_handlers_.getTf(global_frame_,"target_pose",vm_tf) == false)
  {
    std::stringstream sstm;
    sstm << "Failed to get transform between " << global_frame_ << " and " << base_frame_;
    message = sstm.str();
    return false;
  }
  mtk::tf2pose(vm_tf, vm);

  vm.header.stamp = ros::Time::now();
  navigator_.clearCostMaps();

  if(navigator_.moveTo(vm) == false) {
    ROS_ERROR("Navigation Failed while going to vending machine");
    message = "navigation failed....";
    return false;
  }


  // maybe local navi to located on better place..?
  // run autodock algorithm
  playSound("pab.wav");

  return true;
}

bool WaiterIsolated::goToOrigin(std::string& message)
{
  navigator_.backward(0.3);
  navigator_.clearCostMaps();
  // go to in front of vending machine
  geometry_msgs::PoseStamped vm = robot_origin_;
  vm.header.stamp = ros::Time::now();

  if(navigator_.moveTo(vm) == false) {
    ROS_ERROR("Failed to move");
    message = "navigation failed...";
    return false;
  }

  playSound("kaku.wav");
  return true;
}

bool WaiterIsolated::dockInBase() {
  kobuki_msgs::AutoDockingGoal ad_goal;

  in_docking_ = true;
  ac_autodock_.sendGoal(ad_goal);

  while (ac_autodock_.waitForResult(ros::Duration(3.0)) == false)
  {
    ROS_INFO("Waiter : Waiting for docking...");
  }


  in_docking_ = false;
  if(ac_autodock_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Successfully docked...");
    return true;
  }
  else {
    ROS_ERROR("Failed to dock...");
    return false;
  }
}

void WaiterIsolated::playSound(const std::string& wav_file) {
  if ((wav_file.length() > 0))
    bool return_unused = system(("rosrun waiterbot_bringup play_sound.bash " + resources_path_ + wav_file).c_str());

}
}
