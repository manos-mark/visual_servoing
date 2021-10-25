#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


rospy.init_node('camera_sub')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(2)

image_sub = rospy.Subscriber("/camera/image/compressed", Image, camera_callback)

msg = Twist()
msg.linear.x = 0.5
msg.angular.z = 0.2

while not rospy.is_shutdown(): 
    pub.publish(msg)
    rate.sleep()
