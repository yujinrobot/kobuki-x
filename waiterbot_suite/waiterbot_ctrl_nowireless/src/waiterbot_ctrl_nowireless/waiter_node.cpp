/*
  WaiterIsolated Class

  LICENSE : BSD - https://raw.github.com/yujinrobot/kobuki-x/license/LICENSE

  Author :  Jihoon Lee
  Date   : Dec 2013
 */

#include "waiterbot_ctrl_nowireless/waiter_node.hpp"

namespace waiterbot {
  WaiterIsolated::WaiterIsolated(ros::NodeHandle& n) : 
    nh_(n), 
    navigator_(n),
    ac_autodock_("dock_drive_action", true)
  {  
    initialized_ = false;
    waypointsReceived_ = false;
    inDelivery_ = false;
    init();
  }

  WaiterIsolated::~WaiterIsolated() {  }

  void WaiterIsolated::init()
  {
    // parameters 
    ros::NodeHandle priv_n("~");


    priv_n.param("loc_vm", loc_vm_, DEFAULT_LOC_VM);
    priv_n.param("loc_customer", loc_customer_, DEFAULT_LOC_CUSTOMER);

    // listen to green and red buttons
    sub_digital_input_ = nh_.subscribe("digital_input", 5, & WaiterIsolated::digitalInputCB, this);

    // listen to waypoints
    sub_waypoints_ = nh_.subscribe("waypoints", 5, &WaiterIsolated::waypointsCB, this);
    // listen to drink order message
    sub_drinkorder = nh_.subscribe("drink_order", 1, &WaiterIsolated::drinkOrderCB, this);
  }

  bool WaiterIsolated::isInit() {
    if(waypointsReceived_ && navigator_.isInit())
      return true;
    else
      return false;
  }

  void WaiterIsolated::digitalInputCB(const kobuki_msgs::DigitalInputEvent::ConstPtr& msg)
  {
    ROS_INFO("In digital input Call back");
  }

  void WaiterIsolated::waypointsCB(const yocs_msgs::WaypointList::ConstPtr& msg)
  {
    unsigned int i;

    // wait until it finishes a delivery
    while(inDelivery_) { ros::Duration(1).sleep(); }

    map_wp_.clear();
    for(i = 0; i < msg->waypoints.size(); i++)
    {
      geometry_msgs::PoseStamped ps;

      ps.header = msg->waypoints[i].header;
      ps.pose = msg->waypoints[i].pose;

      map_wp_[msg->waypoints[i].name] = ps;
    }
    
    ROS_INFO("Received %lu waypoints", map_wp_.size());
    waypointsReceived_ = true;
  }

  void WaiterIsolated::drinkOrderCB(const waiterbot_msgs::DrinkOrder::ConstPtr& msg)
  {
    ROS_INFO("Drink Order : %d", msg->drink);

    if(inDelivery_)
    {
      ROS_WARN("Waiter : It is serving drink already. rejecting...");
      return;
    }

    inDelivery_ = true;

    // starts to serve.
    ROS_INFO("Starting the delivery...");
    order_process_thread_ = boost::thread(&WaiterIsolated::processOrder, this, msg->drink);
   // processOrder(msg->drink);
  }

  bool WaiterIsolated::endDelivery(bool success)
  {
    if(success)
    {
      ROS_INFO("Delivery Success");
    }
    else 
    {
      ROS_WARN("Delivery Failed!");
    }
    inDelivery_ = false;
    return true;
  }

  void WaiterIsolated::spin() {
    ros::spin();
  }
}
