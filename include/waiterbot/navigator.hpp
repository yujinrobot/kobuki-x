/*
 * navigator.hpp
 *
 *  Created on: Apr 11, 2013
 *      Author: jorge
 */

#ifndef NAVIGATOR_HPP_
#define NAVIGATOR_HPP_

namespace waiterbot
{

class Navigator
{
public:
  Navigator();
  virtual ~Navigator();

  bool init();

  void dockInBase(geometry_msgs::PoseStamped base_marker);

private:
  double autodocking_distance_

  ros::Publisher issue_goal_pub_;
  ros::Publisher cancel_goal_pub_;
};

} /* namespace waiterbot */
#endif /* NAVIGATOR_HPP_ */
