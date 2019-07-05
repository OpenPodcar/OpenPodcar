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

	def callback_wheelAngleCmd(self,msg): 
		joystickx  = -msg.data / (math.pi/4)   #convert back from angle to virtual joystick -1:1
		if joystickx>=0:
                        linearPos = int(1900+ joystickx*600.)  #right   - linear actuatior position, in pololu ints
                else:   
                        linearPos = int(1900+ joystickx*900.)   #left

		msg_out = Int64()
		msg_out.data = linearPos
		self.pub.publish(msg_out)



if __name__ == '__main__':
	rospy.init_node('wheelAngle2pololu',anonymous=True)
	ic = Node()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

