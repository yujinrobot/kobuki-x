#!/usr/bin/env python
#       
# License: BSD
#   https://raw.github.com/yujinrobot/kobuki-x/license/LICENSE
#
import rospy
import waiterbot_ar_pair_approach

if __name__ == '__main__':

    rospy.init_node('ar_pair_approach')
    approach_behaviour = waiterbot_ar_pair_approach.Node()
    approach_behaviour.spin()
