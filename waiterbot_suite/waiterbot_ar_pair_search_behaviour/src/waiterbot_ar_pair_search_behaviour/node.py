#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_concert/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import std_msgs.msg as std_msgs
import threading

# Local imports
from .rotate import Rotate

##############################################################################
# Role Manager
##############################################################################


class Node(object):
    '''
      Manages connectivity information provided by services and provides this
      for human interactive agent (aka remocon) connections.
    '''
    __slots__ = [
#            'role_and_app_table',  # Dictionary of string : concert_msgs.RemoconApp[]
            '_publishers',
            '_subscribers',
            '_parameters',
            '_spotted_markers',
            '_thread',
            '_rotate'
        ]
    SPOTTED_NONE = 'none'
    SPOTTED_LEFT = 'left'
    SPOTTED_RIGHT = 'right'
    SPOTTED_BOTH = 'both'

    ##########################################################################
    # Initialisation
    ##########################################################################

    def __init__(self):
        self._publishers, self._subscribers = self._setup_ros_api()
        self._parameters = self._setup_parameters()
        self._spotted_markers = Node.SPOTTED_NONE
        self._rotate = Rotate('~cmd_vel')
        self._thread = None

    def _setup_parameters(self):
        param = {}
        #param['yaw_absolute_rate'] = rospy.get_param('~yaw_absolute_rate', 1.2)
        #param['yaw_update_increment'] = rospy.get_param('~yaw_update_increment', 1.2)
        return param

    def _setup_ros_api(self):
        '''
          These are all public topics. Typically that will drop them into the /concert
          namespace.
        '''
        publishers = {}
        publishers['result'] = rospy.Publisher('~result', std_msgs.Bool)
        subscribers = {}
        subscribers['enable'] = rospy.Subscriber('~enable', std_msgs.Bool, self._ros_enable_subscriber)
        subscribers['spotted_markers'] = rospy.Subscriber('~spotted_markers', std_msgs.String, self._ros_spotted_subscriber)
        return (publishers, subscribers)

    ##########################################################################
    # Ros Api Functions
    ##########################################################################

    def _ros_enable_subscriber(self, msg):
        if msg.data:
            if not self._rotate.is_running():
                if self._spotted_markers == Node.SPOTTED_BOTH:
                    self._publishers['result'].publish(std_msgs.Bool(True))
                    return
                elif self._spotted_markers == Node.SPOTTED_LEFT:
                    self._rotate.init(yaw_direction=Rotate.CLOCKWISE)
                else:  # self._spotted_markers == Node.SPOTTED_NONE or RIGHT
                    self._rotate.init(yaw_direction=Rotate.COUNTER_CLOCKWISE)
                self._thread = threading.Thread(self._rotate.execute)
        else:
            self._rotate.stop()
            self._thread.join()
            self._thread = None

    def _ros_spotted_subscriber(self, msg):
        self._spotted_markers = msg.data
        if self._spotted_markers == Node.SPOTTED_BOTH and self._rotate.is_running():
            self._rotate.stop()
            self._thread.join()
            self._publishers['result'].publish(std_msgs.Bool(True))

    ##########################################################################
    # Runtime
    ##########################################################################

    def spin(self):
        '''
          Parse the set of /remocons/<name>_<uuid> connections.
        '''
        rospy.spin()
