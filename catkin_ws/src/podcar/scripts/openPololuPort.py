#!/usr/bin/env python
# license removed for brevity
# Fanta : July 10th 2019
# This script converts PololuCmd example C code to python code

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int64
   
import serial
import time

port = "/dev/ttyPololuCOM" 

ser = serial.Serial(port, 115200)
if(ser.isOpen() == False):
  ser.open()
print("Opened Pololu device\n")

"""
pub = rospy.Publisher("pololuFdbk", String, queue_size=1000)
print("Start publishing\n")


# Reads a variable from the jrk.
# The 'command' argument must be one of the two-byte variable-reading
# commands documented in the "Variable Reading Commands" section of
# the jrk user's guide.

def jrkGetVariable(command):
  print("writing command: " + str(command) + "\n")
  val = ser.write(command) 
  print("writing: \n")

  response = ser.read()
  print("received response: " + str(response) + "\n")
  return response[0] + 256*response[1]


# Gets the value of the jrk's Feedback variable (0-4095).
def jrkGetFeedback():
  return jrkGetVariable(bytes(0xA5));


# Gets the value of the jrk's Target variable (0-4095).
def jrkGetTarget():
  return jrkGetVariable(bytes(0xA3));


# Sets the jrk's Target variable (0-4095).
def jrkSetTarget(target):
  command = bytes(0xC0 + (target & 0x1F), (target >> 5) & 0x7F);
  ser.write(command)


def pidCallback(msg):
  target = msg.data;
  print("msgcallback is " + str(target) + "\n");

  feedback = jrkGetFeedback();
  print("Current Feedback is " + str(feedback) + "\n");

  curTarget = jrkGetTarget();
  printf("Current Target is " + str(curTarget) + "\n");

# newTarget = 1000;  //far left
# newTarget = 2000;  //center OK
# newTarget = 2500;  //far right

  print("Setting Target to " + str(target) + "\n");
  jrkSetTarget(target);

  msg.data = feedback;
  pub.publish(msg_out);



if __name__ == '__main__':

  print('Hello')
  
  rospy.init_node('pololuController', anonymous=True);
  sub = rospy.Subscriber("pololuCmd", Int64, pidCallback);
  rospy.spin();
 
  ser.close();"""



