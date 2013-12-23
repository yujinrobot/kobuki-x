/*
 *  nav_watchdog.hpp
 *
 *  Created on: Apr 10, 2013
 *      Author: jorge
 */

#ifndef NAV_WATCHDOG_HPP_
#define NAV_WATCHDOG_HPP_


#include <ros/ros.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
//#include <move_base_msgs/MoveBaseActionFeedback.h>
//#include <move_base_msgs/MoveBaseActionResult.h>


namespace waiterbot
{

#define LOCALIZED_ARMK   0x0001
#define LOCALIZED_AMCL   0x0002
#define LOCALIZED_FAKE   0x0004   // Ignore whether we are localized (local nav. or mapping


class NavWatchdog
{
public:

  /*********************
  ** Initialization
  **********************/
  NavWatchdog();
  ~NavWatchdog();

  bool init();
  bool localized() { return localized_ != 0; };

  void arMarkerMsgCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

private:
  bool   check_localized_;
  double  amcl_max_error_;
  uint16_t     localized_;

  geometry_msgs::PoseStamped last_amcl_pose_;
  geometry_msgs::PoseStamped last_amcl_init_;

  ros::Subscriber amcl_p_sub_;  /**< Robot's world pose estimated by amcl, with covariance */
  ros::Subscriber init_p_sub_;  /**< Mean and cov. to (re)initialize amcl particle filter  */
  ros::Subscriber feedbk_sub_;  /**< Feedback on current position of the base in the world */
  ros::Subscriber status_sub_;  /**< Status information on the goals sent to the move_base */
  ros::Subscriber result_sub_;  /**< Result is empty for the move_base action */
  ros::Subscriber n_goal_sub_;  /**< A goal for move_base to pursue in the world */

  ros::Publisher init_pose_pub_;
  ros::Publisher cancel_goal_pub_;

  void newGoalMsgCB(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void amclPoseMsgCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  void initPoseMsgCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
//  void feedbackMsgCB(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg);
//  void glResultMsgCB(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg);
//  void glStatusMsgCB(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);

  double sign(double x)                                         { return x < 0.0 ? -1.0 : +1.0; }

  double max(double a, double b)                                { return std::max(a, b); }
  double max(double a, double b, double c)                      { return max(max(a, b), c); }
  double max(double a, double b, double c, double d)            { return max(max(a, b, c), d); }
  double max(double a, double b, double c, double d, double e)  { return max(max(a, b, c, d), e); }
};

} /* namespace waiterbot */

#endif /* NAV_WATCHDOG_HPP_ */
