#!/usr/bin/env python

import rospy
from math import sqrt
from PingPong.msg import BallInfo, MoveComputerPaddle
from PingPong.srv import GameInfo
from paddle import Paddle

class ComputerNode:

	def __init__(self):
		self.action_publisher = rospy.Publisher('move_computer_paddle', MoveComputerPaddle, queue_size=10)
		#Wait until the game node starts running
		print 'Waiting to get game information'
		rospy.wait_for_service('game_info')
		try:
			get_game_info = rospy.ServiceProxy('game_info', GameInfo)
			game_info = get_game_info()
			self.board_length = game_info.board_length
			self.board_width = game_info.board_width
			self.ball_speed = game_info.ball_speed
			self.scale = game_info.scale
			self.computer = Paddle(self.board_width/2, 0)
			print 'Ready to win!'
			#For listening ball movements
			rospy.Subscriber("ball_info", BallInfo, self.move_computer_paddle)
			self.move_msg = MoveComputerPaddle()
		except rospy.ServiceException, e:
			print 'Service call failed: %s'%e

	def publish_action(self, action):
		'''
		This method publishes the best action for the computer
		'''
		self.move_msg.where = action
		print(action)
		self.action_publisher.publish(self.move_msg)

	def euclidean_distance(self, computer_x, computer_y, ball_x, ball_y):
		'''
		This method calculates l2 distance between the center of the computer's paddle and center of the ball
		'''
		return sqrt(pow(computer_x-ball_x, 2) + pow(computer_y-ball_y, 2))

	def move_computer_paddle(self, data):
		ball_x, ball_y, direction = data.x, data.y, data.direction
		action, score = 'stay', self.euclidean_distance(self.computer.x, self.computer.y, ball_x, ball_y)
		#To check if up action can be performed
		if self.computer.x > 1.5*self.scale:
			up_score = self.euclidean_distance(self.computer.x-self.scale, self.computer.y, ball_x, ball_y)
			if up_score < score:
				action = 'up'
				score = up_score
		if self.computer.x < self.board_width-1.5*self.scale:
			down_score = self.euclidean_distance(self.computer.x+self.scale, self.computer.y, ball_x, ball_y)
			if down_score < score:
				action = 'down'
		if action == 'up':
			self.computer.update_location(-self.scale) 
		elif action == 'down':
			self.computer.update_location(self.scale)
		self.publish_action(action) 

if __name__ == '__main__':
	rospy.init_node('computer_node', anonymous=True)
	computer = ComputerNode()
	rospy.spin()