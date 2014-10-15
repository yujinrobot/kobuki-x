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
    rospy.loginfo("Pressing Red button - 1")
    m.values = [True, False, False, False]
    input_pub.publish(m)
    rospy.sleep(0.1)
    rospy.loginfo("Releasing Red button")
    m.values = [True, True, False, False]
    input_pub.publish(m)
    rospy.sleep(0.1)

    rospy.loginfo("Pressing Red button - 2")
    m.values = [True, False, False, False]
    input_pub.publish(m)
    rospy.sleep(0.1)
    rospy.loginfo("Releasing Red button")
    m.values = [True, True, False, False]
    input_pub.publish(m)
    rospy.sleep(0.1)

    rospy.loginfo("Pressing Red button - 3")
    m.values = [True, False, False, False]
    input_pub.publish(m)
    rospy.sleep(0.1)
    rospy.loginfo("Releasing Red button")
    m.values = [True, True, False, False]
    input_pub.publish(m)
    rospy.sleep(0.1)

    rospy.loginfo("Done")
