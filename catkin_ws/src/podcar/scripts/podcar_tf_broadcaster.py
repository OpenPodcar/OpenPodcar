#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import tf
import math

def callback(msg):
    br = tf.TransformBroadcaster()
    quatArray = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                        quatArray,
                        rospy.Time.now(),
                        "base_link",
                        "odom")
    br.sendTransform((0.66, 0, 0.74),
                        tf.transformations.quaternion_from_euler(math.pi/2, 0, -math.pi/2, 'ryxz'),
                        rospy.Time.now(),
                        "kinectLink",
                        "base_link")



# [1/msg.pose.pose.orientation.x, 1/msg.pose.pose.orientation.y, 1/msg.pose.pose.orientation.z, 1/msg.pose.pose.orientation.w]

rospy.init_node("podcar_tf_broadcaster", anonymous=True)
rospy.Subscriber("odometry/groundTruth", Odometry, callback, queue_size=1)
rospy.spin()