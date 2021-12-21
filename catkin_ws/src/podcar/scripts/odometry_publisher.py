#!/usr/bin/env python

### Author: Fanta Camara Dec 2021
### For odometry/groundTruth publisher
### inspired by: https://gist.github.com/atotto/f2754f75bedb6ea56e3e0264ec405dcf

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Float64


PODCAR_ANGULAR_SPEED = 0.175   # podcar turns about 0.175 rad/s ~= 10deg/sec

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
		
		self.dt = 0.
		self.delta_th =  0.
		
		self.target_angle = 0.

		# subscribe to speed_meterssec & wheelAngleCmd
		self.speed_sub = rospy.Subscriber("speedcmd_meterssec",Float64, self.callback_speed, queue_size=1)
		self.angle_sub = rospy.Subscriber("wheelAngleCmd",Float64, self.callback_angle, queue_size=10) 

	# compute odometry in a typical way given the velocities of the robot

	def callback_speed(self, msg):
	
		self.vx = msg.data	  # linear x velocity
		
		if self.vx > 0:
			self.vx = self.vx/15.0   # podcar moves 0.2m/s forward
		else:
			self.vx = self.vx/5.0	 # podcar moves 0.2m/s backward
			
	
	def callback_angle(self, msg):
	
		self.target_angle = msg.data
		if abs(self.target_angle - self.th) > 0.01 :
			if self.target_angle > self.th:  # anticlockwise
				self.vth = PODCAR_ANGULAR_SPEED
			else:                   # clockwise
				self.vth = - PODCAR_ANGULAR_SPEED
		else:
			self.vth = 0.0
		

if __name__ == '__main__':
	
	rospy.init_node('odometry_publisher')
	
	odom_pub = rospy.Publisher("odometry/groundTruth", Odometry, queue_size=1000)
	odom_broadcaster = tf.TransformBroadcaster()
	
	node = OdomNode()

	r = rospy.Rate(10)
	while not rospy.is_shutdown():
	
		node.current_time = rospy.Time.now()
		node.dt = (node.current_time - node.last_time).to_sec()
		
		delta_x = (node.vx * cos(node.th) - node.vy * sin(node.th)) * node.dt
		delta_y = (node.vx * sin(node.th) + node.vy * cos(node.th)) * node.dt
		
		node.x += delta_x
		node.y += delta_y
		
		if node.target_angle != node.th:
			node.delta_th = node.vth * node.dt
			node.th += node.delta_th
		
		# since all odometry is 6DOF we'll need a quaternion created from yaw
		odom_quat = tf.transformations.quaternion_from_euler(0, 0, node.th)
		
		# first, we'll publish the transform over tf
		odom_broadcaster.sendTransform((node.x, node.y, 0.), odom_quat, node.current_time, "odom", "map")
		
		# next, we'll publish the odometry message over ROS
		odom = Odometry()
		odom.header.stamp = node.current_time
		odom.header.frame_id = "map"
		
		# set the position
		odom.pose.pose = Pose(Point(node.x, node.y, 0.), Quaternion(*odom_quat))
		
		# set the velocity
		odom.child_frame_id = "odom"
		odom.twist.twist = Twist(Vector3(node.vx, node.vy, 0), Vector3(0, 0, node.vth))
		
		# publish the message
		odom_pub.publish(odom)
		
		node.last_time = node.current_time
		r.sleep()
