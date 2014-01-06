#
# License: BSD
#   https://raw.github.com/yujinrobot/kobuki-x/license/LICENSE
#

import dynamic_reconfigure.client
import geometry_msgs.msg as geometry_msgs
import rospy
import std_msgs.msg as std_msgs
import tf

class InitPoseManager(object):
    """
        Uses poses from a pose tracker to initialise the global robot pose
        using a pose initialisation node such as fake_localization
    """

    def __init__(self):
        self._initialise = False
        self._sleep_time = rospy.Duration(1 / rospy.get_param('~spin_freq', 10))
        self._simulation = rospy.get_param('~simulation', False)
        if self._simulation:
            rospy.loginfo("InitPoseManager: Running in simulation mode.")

        self._sub_init = rospy.Subscriber('~initialise', std_msgs.Empty, self._initCB)
        self._pub_result = rospy.Publisher('~initialised', std_msgs.Empty, latch=True)
        if not self._simulation:
            self._client = dynamic_reconfigure.client.Client(rospy.get_param('~pose_tracker', 'ar_track_alvar'))
            self._sub_tracked_poses = rospy.Subscriber('pose_tracker/poses', geometry_msgs.PoseWithCovarianceStamped,
                                                       self._trackedPosesCB)
        self._pub_init_pose = rospy.Publisher('fake_localization/initialpose', geometry_msgs.PoseWithCovarianceStamped,
                                              latch=True)

    def _trackedPosesCB(self, msg):
        if self._initialise:
            # send pose to pose initialisation node
            msg.header.stamp -= rospy.Duration(0.2) # TODO: get latest common time
            self._pub_init_pose.publish(msg)
            empty_msg = std_msgs.Empty()
            # disable the pose tracker
            params = { 'enabled' : 'False' }
            config = self._client.update_configuration(params)
            # send result
            self._pub_result.publish(empty_msg)
            rospy.loginfo("InitPoseManager: Initialisation done.")
            self._initialise = False

    def _initCB(self, msg):
        rospy.loginfo("InitPoseManager: Initialisation started.")
        # enable the pose tracker
        if self._simulation:
            pose_msg = geometry_msgs.PoseWithCovarianceStamped()
            pose_msg.header.frame_id = "ar_global"
            pose_msg.header.stamp = rospy.Time.now() - rospy.Duration(0.2) # TODO: get latest common time
            pose_msg.pose.pose.position.x = 1.0
            pose_msg.pose.pose.position.y = 0.0
            pose_msg.pose.pose.position.z = 0.0
            quat = tf.transformations.quaternion_from_euler(0, 0, 3.1416)
            pose_msg.pose.pose.orientation = geometry_msgs.Quaternion(*quat)
            self._pub_init_pose.publish(pose_msg)
            # send success right away
            empty_msg = std_msgs.Empty()
            self._pub_result.publish(empty_msg)
            rospy.loginfo("InitPoseManager: Initialisation done.")
        else:
            params = { 'enabled' : 'True' }
            config = self._client.update_configuration(params)
            self._initialise = True


    def spin(self):
        while not rospy.is_shutdown():
            rospy.sleep(self._sleep_time)
