#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

def callback(msg):
    odomPub.publish(msg)

rospy.init_node("odometry_handler", anonymous=True)

odomSub = rospy.Subscriber("odometry/groundTruth", Odometry, callback, queue_size=1)
odomPub = rospy.Publisher("nav_msgs/Odometry", Odometry, callback, queue_size=1)

rospy.spin()