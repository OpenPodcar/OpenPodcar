#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import tf

def callback(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0),
                        tf.transformations.quaternion_from_euler(0, 0, msg.pose.pose.orientation.w),
                        rospy.Time.now(),
                        "podcar",
                        "world")
    br.sendTransform((0.66, 0, 0.74),
                        (0, 0, 0, 1),
                        rospy.Time.now(),
                        "kinectLink",
                        "podcar")

# [1/msg.pose.pose.orientation.x, 1/msg.pose.pose.orientation.y, 1/msg.pose.pose.orientation.z, 1/msg.pose.pose.orientation.w]

rospy.init_node("podcar_tf_broadcaster", anonymous=True)
rospy.Subscriber("odometry/groundTruth", Odometry, callback, queue_size=1)
rospy.spin()