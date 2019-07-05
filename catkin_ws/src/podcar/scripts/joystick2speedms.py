#!/usr/bin/env python
import sys
import rospy
import pdb
from podcar.msg  import Joystick
from std_msgs.msg import Float64
import subprocess, time

#TODO use standard Twist messages

def rescale(x, oldmin, oldmax, newmin, newmax):
	r = (x-oldmin)/(oldmax-oldmin)
	out = newmin + r*(newmax-newmin)
	return out

class Node:
	def __init__(self):
		now = rospy.get_rostime()
		self.received_reponse_to_last_command = True

		self.pub = rospy.Publisher('speedcmd_meterssec', Float64, queue_size=10)
		self.sub = rospy.Subscriber("joystick",Joystick,self.callback_joystick, queue_size=1)
		self.buffer = []
		for i in range(0,100):
			self.buffer.append('X')
		self.buffer_idx_next = 0
		rospy.loginfo("initialised speed node")

	def callback_joystick(self,data): #convert the joystick state to desired velocity 
		joysticky = -data.y
		rosdeadzonemin = -.2   #deadzone joystick positions - to allow turnign of front wheels while stationary without small back motions
		rosdeadzonemax =  .2
		maxspeed_meter_per_second_fwd = 3.0
		maxspeed_meter_per_second_bkwd = 1.0
		if joysticky>rosdeadzonemin and joysticky<rosdeadzonemax:
			velocity = 0.
		elif joysticky > 0 :
			velocity = rescale( joysticky, rosdeadzonemax, 1.,   0., maxspeed_meter_per_second_fwd)
		elif joysticky < 0: 	
			velocity = rescale( joysticky, -1., rosdeadzonemin,  -maxspeed_meter_per_second_bkwd, 0.)	
#		print((data.y, velocity))
                msg_out = Float64()
                msg_out.data = velocity
                self.pub.publish(msg_out)

if __name__ == '__main__':
	rospy.init_node('joystick2speedms',anonymous=True)
	ic = Node()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

