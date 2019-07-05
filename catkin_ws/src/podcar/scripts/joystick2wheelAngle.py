#!/usr/bin/env python
import sys
import rospy
import pdb
from podcar.msg  import Joystick
from std_msgs.msg import Float64
import subprocess, time
import math

class Node:
	def __init__(self):
		now = rospy.get_rostime()
		self.pub = rospy.Publisher('wheelAngleCmd', Float64, queue_size=10)
		self.sub = rospy.Subscriber("joystick",Joystick,self.callback_joystick, queue_size=1)

	def callback_joystick(self,data): 
		theta = -(math.pi/4)* data.x
		msg_out = Float64()
		msg_out.data = theta
		self.pub.publish(msg_out)



if __name__ == '__main__':
	rospy.init_node('joystick2wheelAngle',anonymous=True)
	ic = Node()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

