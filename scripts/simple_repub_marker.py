#!/usr/bin/env python3

from yaml.error import Mark
import rospy
from visualization_msgs.msg import MarkerArray

pub = rospy.Publisher('/occ_cells', MarkerArray, queue_size=10)

def callback(data):
    copy = data
    for i in copy.markers:
        i.header.frame_id = "map"
        # print(str(i.header.frame_id))
    pub.publish(copy)


rospy.init_node('marker_repub', anonymous=False)
rospy.Subscriber("/occupied_cells_vis_array", MarkerArray, callback)

rospy.spin()

