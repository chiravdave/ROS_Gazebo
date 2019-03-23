#!/usr/bin/env python

import rospy
from math import sqrt
from numpy.random import choice
from PingPong.msg import BallInfo, MovePlayerPaddle
from PingPong.srv import GameInfo
from paddle import Paddle

class PlayerNode:

	def __init__(self):
		self.action_publisher = rospy.Publisher('move_player_paddle', MovePlayerPaddle, queue_size=10)
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
			self.player = Paddle(self.board_width/2, self.board_length)
			print 'Ready to win!'
			#For listening ball movements
			rospy.Subscriber("ball_info", BallInfo, self.move_player_paddle)
			self.move_msg = MovePlayerPaddle()
		except rospy.ServiceException, e:
			print 'Service call failed: %s'%e

	def publish_action(self, action):
		'''
		This method publishes the best action for the player
		'''
		self.move_msg.where = action
		print(action)
		self.action_publisher.publish(self.move_msg)

	def euclidean_distance(self, player_x, player_y, ball_x, ball_y):
		'''
		This method calculates l2 distance between the center of the player's paddle and center of the ball
		'''
		return sqrt(pow(player_x-ball_x, 2) + pow(player_y-ball_y, 2))

	def move_player_paddle(self, data):
		action = choice(['up', 'down'])
		if action == 'up':
			self.player.update_location(-self.scale)
		elif action == 'down':
			self.player.update_location(self.scale)
		self.publish_action(action) 

if __name__ == '__main__':
	rospy.init_node('player_node', anonymous=True)
	player = PlayerNode()
	rospy.spin()