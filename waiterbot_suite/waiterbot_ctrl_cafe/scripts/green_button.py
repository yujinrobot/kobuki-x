#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_demos/license/LICENSE
#

import rospy
import sys
import kobuki_msgs.msg

if __name__ == '__main__':
    rospy.init_node('dummy_buttons')

    input_pub = rospy.Publisher('/mobile_base/events/digital_input', kobuki_msgs.msg.DigitalInputEvent)
    rospy.sleep(0.5)

    m = kobuki_msgs.msg.DigitalInputEvent()
    rospy.loginfo("Pressing Green button")
    m.values = [False, True, False, False]
    input_pub.publish(m)
    rospy.sleep(0.5)
    rospy.loginfo("Releasing Green button")
    m.values = [True, True, False, False]
    input_pub.publish(m)
    rospy.sleep(0.5)

    rospy.loginfo("Done")
