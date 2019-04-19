#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
from numpy import radians

def move_arm():
	robot = moveit_commander.RobotCommander()
	group_name = "panda_arm"
	move_group = moveit_commander.MoveGroupCommander(group_name)
	'''
	planning_frame = move_group.get_planning_frame()
	print "============ Planning frame: %s" % planning_frame
	'''
	#print move_group.get_end_effector_link()
	#Taking coordinate values of the object 
	object_x, object_y, object_z = 0.5, 0.5, 0.5
	#Initializing offset values for translation
	offset_x, offset_y, offset_z = 0, 0, 0.2
	#Initializing offset values for rotation
	roll, pitch, yaw = 0, 0, 0
	#Adding translation along x-axis 
	final_x = object_x + offset_x
	#Adding translation along y-axis
	final_y = object_y + offset_y
	#Adding translation along z-axis
	final_z = object_z + offset_z
	#Applying rotation to the arm

	for i in range(0, 2, 10):
		final_quaternion = quaternion_from_euler(radians(roll-i), radians(pitch), radians(yaw), 'sxyz')
		pose_goal = geometry_msgs.msg.Pose()
		pose_goal.position.x = final_x
		pose_goal.position.y = final_y
		pose_goal.position.z = final_z
		pose_goal.orientation.x = final_quaternion[0]
		pose_goal.orientation.y = final_quaternion[1]
		pose_goal.orientation.z = final_quaternion[2]
		pose_goal.orientation.w = final_quaternion[3]
		move_group.set_pose_target(pose_goal)
		plan = move_group.go(wait=True)
		if plan == True:
			print roll+i
			rospy.sleep(20)
		print 'Result is: %s' % plan
		move_group.stop()
		move_group.clear_pose_targets()

if __name__ == '__main__':
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('move_arm', anonymous=True)
	move_arm()