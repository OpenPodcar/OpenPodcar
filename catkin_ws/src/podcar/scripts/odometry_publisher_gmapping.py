#!/usr/bin/env python

### Author: Fanta Camara Dec 2021
### For odometry/groundTruth publisher
### inspired by: https://gist.github.com/atotto/f2754f75bedb6ea56e3e0264ec405dcf
### and http://wiki.ros.org/turtlesim/Tutorials/Rotating%20Left%20and%20Right

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Float64
from std_msgs.msg import Int64


class OdomNode:
	def __init__(self):
		
		self.current_time = rospy.Time.now()
		self.last_time = rospy.Time.now()
		self.x = 0.
		self.y = 0.
		self.th = 0.
		
		self.vx = 0.
		self.vth = 0.  
		
		self.dt = 0.
		self.wheelbase = 1.05
		self.current_angle = 0.

		# subscribe to speedcmd_meterssec & wheelAngleCmd
		self.speed_sub = rospy.Subscriber("speedcmd_meterssec",Float64, self.callback_speed, queue_size=1)
		#self.wheelangle_sub = rospy.Subscriber("wheelAngleCmd",Float64, self.callback_wheelangle, queue_size=1) 
		self.fdkangle_sub = rospy.Subscriber("pololuFdbk",Int64, self.callback_fdkangle, queue_size=1) 

	# compute odometry in a way that reflfects the podcar's behaviour
	def callback_speed(self, msg):
		self.vx = msg.data/8.  # linear x velocity  # podcar moves between 0.1m/s (slow) and 0.2m/s (fast)
			
	#def callback_wheelangle(self, msg):
		#self.th = -msg.data
		
					
	def callback_fdkangle(self, msg): 
		feedback = msg.data  # to be converted back to angle
		if feedback > 1879:    
			self.current_angle = (math.pi/4) * (feedback - 1879) / 300. # turning on the left
		elif feedback < 1870:
			self.current_angle = (math.pi/4) * (feedback - 1870) / 200. # turning on the right
		else:
			self.current_angle = 0.
		

if __name__ == '__main__':
	
	rospy.init_node('odometry_publisher')
	
	odom_pub = rospy.Publisher("odometry/groundTruth", Odometry, queue_size=1)
	
	node = OdomNode()

	r = rospy.Rate(5)
	while not rospy.is_shutdown():
	
		node.current_time = rospy.Time.now()
		node.dt = (node.current_time - node.last_time).to_sec()
		
		node.vth = node.vx * math.tan(node.current_angle) / node.wheelbase
		
		print("Current angle: " + str(node.current_angle) + "\n")
		
		delta_th = node.vth*node.dt  
		print("Delta_th: " + str(delta_th) + "\n")
		#delta_th = 0.0
		#node.th = node.current_angle 
	
		#delta_x = (node.vx * cos(node.th + delta_th/2)) * node.dt
		#delta_y = (node.vx * sin(node.th + delta_th/2)) * node.dt
		
		delta_x = (node.vx * cos(node.th)) * node.dt
		delta_y = (node.vx * sin(node.th)) * node.dt
		
		node.x += delta_x
		node.y += delta_y
		
		node.th += delta_th
		print("Theta: " + str(node.th) + "\n")
		
		#node.th = node.current_angle
		
		# since all odometry is 6DOF we'll need a quaternion created from yaw
		odom_quat = tf.transformations.quaternion_from_euler(0., 0., node.th/2)
		
		# next, we'll publish the odometry message over ROS
		odom = Odometry()
		odom.header.stamp = node.current_time
		odom.header.frame_id = "odom"
		
		# set the position
		odom.pose.pose = Pose(Point(node.x, node.y, 0.), Quaternion(*odom_quat))
		
		# set the velocity
		odom.child_frame_id = "base_link"
		#odom.twist.twist = Twist(Vector3(node.vx, 0, 0), Vector3(0, 0, node.vth))
		
		# publish the message
		odom_pub.publish(odom)
		
		node.last_time = node.current_time
		r.sleep()
