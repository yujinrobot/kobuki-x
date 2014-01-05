#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/yujinrobot/kobuki-x/license/LICENSE
#

import rospy
import waiterbot_ctrl_nowireless

if __name__ == '__main__':
  rospy.init_node('init_pose_manager')
  init_pose_manager = waiterbot_ctrl_nowireless.InitPoseManager()
  init_pose_manager.spin()
