#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan


def callback(msg):
    sensorPub.publish(msg)

rospy.init_node("sensor_msgs_handler", anonymous=True)

cameraSub = rospy.Subscriber("camera/depth/points", PointCloud2, callback, queue_size=1)
sensorPub = rospy.Publisher("sensor_msgs/PointCloud", PointCloud2, queue_size=1)
laserScanSub = rospy.Subscriber("sensor_msgs/LaserScan", LaserScan, queue_size=1)

rospy.spin()