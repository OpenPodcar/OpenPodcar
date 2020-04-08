#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

def callback(msg):
    forwardSpeed, wheelAngle = Float64(), Float64()
    forwardSpeed.data = msg.linear.x
    wheelAngle.data = msg.angular.z

    forwardPub.publish(forwardSpeed)
    anglePub.publish(wheelAngle)

rospy.init_node("cmd_vel_handler", anonymous=True)

twistSub = rospy.Subscriber("cmd_vel", Twist, callback, queue_size=1)
forwardPub = rospy.Publisher("speedcmd_meterssec", Float64, queue_size=1)
anglePub = rospy.Publisher("wheelAngleCmd", Float64, queue_size=1)

rospy.spin()