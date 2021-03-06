#!/usr/bin/env python
import sys
import rospy
import pdb
from std_msgs.msg import Float64
from std_msgs.msg import Int64
import subprocess, time
import math

class Node:
	def __init__(self):
		now = rospy.get_rostime()
		self.pub = rospy.Publisher('pololuCmd', Int64, queue_size=10)
		self.sub = rospy.Subscriber("wheelAngleCmd",Float64,self.callback_wheelAngleCmd, queue_size=1)
		
		# initialise linear actuator to zero degree (i.e. 1900 increments)
		self.linearPos = 1900
		

	def callback_wheelAngleCmd(self,msg): 
		angle = -msg.data # suppose move_base angle cmd are between -pi/2 and pi/2, we want instead -pi/4 and pi/4
		joystickx = angle / (math.pi/4)   #convert back from angle to virtual joystick -1:1
		#joystickx = angle/2
		
		if joystickx>=0:
			ic.linearPos = int(1900 + joystickx*600.)  # right   - linear actuatior position, in pololu ints
		else:
			ic.linearPos = int(1900 + joystickx*900.)   # left

		msg_out = Int64()
		
		if ic.linearPos < 1001:
			ic.linearPos = 1001
		elif ic.linearPos > 2499:
			ic.linearPos = 2499
		msg_out.data = ic.linearPos
		self.pub.publish(msg_out)

if __name__ == '__main__':
	rospy.init_node('wheelAngle2pololu',anonymous=True)
	ic = Node()
	
	initial_pos = Int64()
	initial_pos.data = 1900
	ic.pub.publish(initial_pos.data)
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

