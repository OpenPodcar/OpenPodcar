#!/usr/bin/env python


### For odometry/groundTruth publisher
### original source: https://gist.github.com/atotto/f2754f75bedb6ea56e3e0264ec405dcf

#!/usr/bin/env python

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Float64


class OdomNode:
	def __init__(self):
		
		self.current_time = rospy.Time.now()
		self.last_time = rospy.Time.now()
		self.x = 0.
		self.y = 0.
		self.th = 0.
		
		self.vx = 0.
		self.vy = 0.
		self.vth = 0.

		self.dt = (self.current_time - self.last_time).to_sec()
	
		# podcar velocities

		# subscribe to speed_meterssec & wheelAngleCmd
		self.speed_sub = rospy.Subscriber("speedcmd_meterssec",Float64, self.callback_speed, queue_size=1)
		self.angle_sub = rospy.Subscriber("wheelAngleCmd",Float64, self.callback_angle, queue_size=10) 

# compute odometry in a typical way given the velocities of the robot

	def callback_speed(self, msg):
	
		self.vx = msg.data	  # linear x velocity
		
		delta_x = (self.vx * cos(self.th) - self.vy * sin(self.th)) * self.dt
		delta_y = (self.vx * sin(self.th) + self.vy * cos(self.th)) * self.dt
		
		self.x += delta_x
		self.y += delta_y
	
	
	def callback_angle(self, msg):
		
		self.vth = msg.data/self.dt  # angular velocity on z axis
		
		delta_th = self.vth * self.dt
		
		self.th += delta_th
		

if __name__ == '__main__':
	
	rospy.init_node('odometry_publisher')
	
	odom_pub = rospy.Publisher("odometry/groundTruth", Odometry, queue_size=10)
	odom_broadcaster = tf.TransformBroadcaster()
	
	node = OdomNode()

	r = rospy.Rate(1.0)
	while not rospy.is_shutdown():
		node.current_time = rospy.Time.now()
		
		# since all odometry is 6DOF we'll need a quaternion created from yaw
		odom_quat = tf.transformations.quaternion_from_euler(0, 0, node.th)
		
		# first, we'll publish the transform over tf
		odom_broadcaster.sendTransform((node.x, node.y, 0.), odom_quat, node.current_time, "/base_link", "/odom")
		
		# next, we'll publish the odometry message over ROS
		odom = Odometry()
		odom.header.stamp = node.current_time
		odom.header.frame_id = "/odom"
		
		# set the position
		odom.pose.pose = Pose(Point(node.x, node.y, 0.), Quaternion(*odom_quat))
		
		# set the velocity
		odom.child_frame_id = "/base_link"
		odom.twist.twist = Twist(Vector3(node.vx, node.vy, 0), Vector3(0, 0, node.vth))
		
		# publish the message
		odom_pub.publish(odom)
		
		node.last_time = node.current_time
		r.sleep()
