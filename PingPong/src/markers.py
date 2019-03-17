#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker

def send_marker():
	rospy.init_node('send_marker', anonymous=True)
	publisher = rospy.Publisher('visualization_marker', Marker, queue_size=1)
	shape = Marker.CUBE
	while not rospy.is_shutdown():
		marker = Marker()
		marker.header.frame_id = "/map"
		marker.header.stamp = rospy.Time.now()
		marker.action = marker.ADD
		marker.type = shape
		marker.ns = 'ball'
		marker.id = 1
		marker.scale.x = 0.6
		marker.scale.y = 0.2
		marker.scale.z = 0.05
		marker.color.a = 1
		marker.color.r = 1
		marker.color.g = 1
		marker.color.b = 1
		marker.pose.orientation.w = 0
		marker.pose.orientation.x = 0
		marker.pose.orientation.y = 0
		marker.pose.orientation.z = 0
		marker.pose.position.x = 0.5
		marker.pose.position.y = 0.1
		marker.pose.position.z = 0.025 
		publisher.publish(marker)
		shape = marker.CUBE
		rospy.sleep(3)

if __name__ == '__main__':
	send_marker()