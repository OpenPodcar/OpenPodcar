#!/usr/bin/env python

### Author: Fanta Camara Jan 2022
### For odometry/groundTruth publisher

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, Pose2D
from std_msgs.msg import Float64
from std_msgs.msg import Int64


class OdomNode:
	def __init__(self):
		
		self.x = 0.
		self.y = 0.
		self.th = 0.

		# subscribe to pose2D from laser_scan_matcher
		self.speed_sub = rospy.Subscriber("pose2D", Pose2D, self.callback_odom_laser, queue_size=1)
		self.broadcaster = tf.TransformBroadcaster()


	# compute odometry in a way that reflfects the podcar's behaviour
	def callback_odom_laser(self, msg):
		self.x = msg.x 
		self.y = msg.y
		self.th = msg.theta
		
		# since all odometry is 6DOF we'll need a quaternion created from yaw
		odom_quat = tf.transformations.quaternion_from_euler(0., 0., node.th)
		
		current_time = rospy.Time.now()
		
		# first, we'll publish the transform over tf
		#self.broadcaster.sendTransform((self.x, self.y, 0.), odom_quat, current_time, "base_link", "odom")
		
		
		# next, we'll publish the odometry message over ROS
		odom = Odometry()
		odom.header.stamp = current_time
		odom.header.frame_id = "odom"
		
		# set the position
		odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))
		
		# set the velocity
		odom.child_frame_id = "base_link"
		
		# publish the message
		odom_pub.publish(odom)
		


if __name__ == '__main__':
	
	rospy.init_node('odometry_publisher_laser_scan')
	
	odom_pub = rospy.Publisher("odometry/groundTruth", Odometry, queue_size=1)
	
	node = OdomNode()

	rospy.spin()

