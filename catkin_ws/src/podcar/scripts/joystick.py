#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math
from podcar.msg  import Joystick

import pygame

#to supress SDL printing to console
from contextlib import contextmanager
import sys, os
@contextmanager
def suppress_stdout():
    with open(os.devnull, "w") as devnull:
        old_stdout = sys.stdout
        old_stderr = sys.stderr
        sys.stdout = devnull
        sys.stderr = devnull
        try:  
            yield
        finally:
            sys.stdout = old_stdout
            sys.stderr = old_stderr


#hold joystick diagonally, so x=left track speed, y=right track.

class Controller:
        
	def callbackObsDet(self, data):
		self.dataObsDet=data
		self.process()
	def callbackEkf(self, data):
		self.dataEkf=data
		self.process()

	def process(self):

		for event in pygame.event.get(): # User did something          POLLING QUEUE
			if event.type == pygame.QUIT: # If user clicked close
				done=True  #not used
							
		#with suppress_stdout(): #not working - trying to rmove SDL debug cmds. TODO recompile SDL wout debug info.
		if 1:
			joystick = pygame.joystick.Joystick(0)
			joystick.init()
			msg = Joystick(joystick.get_axis(0), joystick.get_axis(1), joystick.get_axis(2), joystick.get_axis(3), joystick.get_button(0), joystick.get_button(1), joystick.get_button(2), joystick.get_button(3), joystick.get_button(4), joystick.get_button(5), joystick.get_button(6), joystick.get_button(7), joystick.get_button(8), joystick.get_button(9), joystick.get_button(10), joystick.get_button(11),0,0,0, 0 )
			self.pub.publish(msg)

	def __init__(self):
		self.pub = rospy.Publisher("joystick",Joystick, queue_size=20)
		rospy.init_node('joystick', anonymous=True)

	
		pygame.init()
		pygame.joystick.init()

		rate  = rospy.Rate(10)              #rate in Hz
		if 1:
				while (1):
					self.process()
					rate.sleep()

if __name__ == '__main__':
    try:
        c = Controller()
    except rospy.ROSInterruptException:
	print "ex"
        pass
