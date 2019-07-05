#!/usr/bin/env python
import sys
import rospy
import pdb
from podcar.msg  import Joystick
import subprocess, time


class Node:
	def __init__(self):
		now = rospy.get_rostime()
		self.sub = rospy.Subscriber("/podcar/joystick",Joystick,self.callback_joystick, queue_size=1)
	def callback_joystick(self,data): 
		print(data.x, data.y)
		
		#x,y are -1:+1.  used to have to run as sudo but not given access.
		self.sendSerialCommandToSteering(data.x)

	def sendSerialCommandToSteering(self, joystickx):  #TODO use proper desired angle etc. TODO return feedback message.

		print(joystickx)
#		joystickx=-joystickx
		#900=extreme L;  2000 straight;  2400 extreme R.
		if joystickx>=0:
			linearPos = int(1900+ joystickx*600.)  #right
		else:
			linearPos = int(1900+ joystickx*900.)   #left
		cmd = "/home/podcar/catkin_ws/src/podcar/tools/JrkCmd/JrkCmd --target %i"%linearPos  #using Pololu's closed cmd line tool
		print(cmd)
		subprocess.call(cmd, shell=True)		



if __name__ == '__main__':
	rospy.init_node('joystick2steer',anonymous=True)
	ic = Node()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

