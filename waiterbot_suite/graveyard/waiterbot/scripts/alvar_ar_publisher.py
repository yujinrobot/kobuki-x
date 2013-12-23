#!/usr/bin/env python

import rospy
import yaml

from geometry_msgs.msg import *
from rospy_message_converter import message_converter
from ar_track_alvar.msg import AlvarMarkers, AlvarMarker

def publish(filename):
    yaml_data = None 
    with open(filename) as f:
       yaml_data = yaml.load(f)

    ar_list = AlvarMarkers()

    for m in yaml_data:
        ar = AlvarMarker()
        ar.id = m['id']
        ar.confidence = m['confidence']
        ar.pose.header.frame_id = m['frame_id']
        ar.pose.header.stamp = rospy.Time.now()
        ar.pose.pose = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',m['pose'])
        ar_list.markers.append(ar)
    
    ar_pub.publish(ar_list)
    
    return
    


if __name__ == '__main__':
    rospy.init_node('ar_loader')
    filename = rospy.get_param('~filename')

    ar_pub = rospy.Publisher('marker_pose_list',AlvarMarkers,latch=True)
    rospy.loginfo('Publishing AR markers.')
    publish(filename)
    rospy.spin()
