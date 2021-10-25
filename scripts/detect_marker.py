#! /usr/bin/env python

import rospy
from aruco_pose.msg import MarkerArray
rospy.init_node('my_node')

def markers_callback(msg):
    print('Detected markers:')
    for marker in msg.markers:
        print('Marker: %s' % marker)

# Create a Subscription object. Each time a message is posted in aruco_detect/markers, the markers_callback function is called with this message as its argument.
rospy.Subscriber('aruco_detect/markers', MarkerArray, markers_callback)

# ...
while not rospy.is_shutdown(): 
    rospy.spin()