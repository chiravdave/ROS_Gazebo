#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker

def send_marker():
	rospy.init_node('send_marker', anonymous=True)
	publisher = rospy.Publisher('visualization_marker', Marker, queue_size=1)
	rospy.Rate(1).sleep()
	marker = Marker()
	marker.header.frame_id = "/map"
	marker.header.stamp = rospy.Time.now()
	marker.action = marker.ADD
	marker.type = Marker.TEXT_VIEW_FACING
	marker.ns = 'text'
	marker.id = 1
	marker.scale.x = 0.3
	marker.scale.y = 0.3
	marker.scale.z = 0.3
	marker.color.a = 1
	marker.color.r = 1
	marker.color.g = 1
	marker.color.b = 1
	marker.pose.orientation.w = 0
	marker.pose.orientation.x = 0
	marker.pose.orientation.y = 0
	marker.pose.orientation.z = 0
	marker.pose.position.x = 0.3
	marker.pose.position.y = 2
	marker.pose.position.z = 0.5
	marker.text = '2' 
	publisher.publish(marker)

if __name__ == '__main__':
	send_marker()