#!/usr/bin/env python

import rospy
import yaml
import tf
import copy

from geometry_msgs.msg import *
from rospy_message_converter import message_converter
from visualization_msgs.msg import *
from yocs_msgs.msg import *

def publish(filename):
    yaml_data = None 
    with open(filename) as f:
       yaml_data = yaml.load(f)

    table_list  = TableList()
    marker_list = MarkerArray()    

    marker_id = 1
    for t in yaml_data:

        # Tables
        table = Table()
        table.name = t['name']
        table.radius = float(t['radius'])
        table.height = float(t['height'])
        table.pose.header.frame_id = t['frame_id']
        table.pose.header.stamp = rospy.Time.now()
        table.pose.pose.pose = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose',t['pose'])
        table_list.tables.append(table)
        
        # Markers
        marker = Marker()
        marker.id = marker_id
        marker.header = table.pose.header
        marker.type = Marker.CYLINDER
        marker.ns = "tables"
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration.from_sec(0)
        marker.pose = copy.deepcopy(table.pose.pose.pose)
        marker.pose.position.z += table.height/2.0
        marker.scale.x = table.radius * 2
        marker.scale.y = table.radius * 2  
        marker.scale.z = table.height
        marker.color.r = 0
        marker.color.g = 0
        marker.color.b = 1.0
        marker.color.a = 0.5
                                                                                                                                                    
        marker_list.markers.append(marker)
                                                                                                                                                    
        marker_id = marker_id + 1

    table_pub.publish(table_list)
    marker_pub.publish(marker_list)
    
    return

if __name__ == '__main__':
    rospy.init_node('table_loader')
    filename = rospy.get_param('~filename')
    
    marker_pub = rospy.Publisher('table_marker', MarkerArray, latch = True)
    table_pub = rospy.Publisher('table_pose_list', TableList, latch = True)
    
    rospy.loginfo('Publishing tables and their visualization markers.')
    publish(filename)
    rospy.spin()
