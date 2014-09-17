#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/yujinrobot/kobuki-x/license/LICENSE

import copy
import rospy
import kobuki_msgs.msg import DigitalInputEvent

SUB_DIGITAL_INPUT = '~digital_input'

def check_button_event(prev, curr):

    green = False
    red = False
    if prev.values[0] is False and curr.values[0] is True:
        green = True

    if prev.values[1] is False and curr.values[1] is True:
        red = True 

    return green, red

class ButtonControl(object):

    def __init__(self, callback):
        self.sub = rospy.Subscriber(SUB_DIGITAL_INPUT, DigitalInputEvent, self._process_digital_input)
        self._previous_button None
        self.callback = callback

    def _process_digital_input(self, msg):
        if self._previous_button:
            green, red = check_button_event(self._previous_button, msg)
            self.callback(green, red)
        self._previous_button = copy.deepcopy(msg)
