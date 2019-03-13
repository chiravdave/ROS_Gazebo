#!/usr/bin/env python

from numpy.random import choice
from visualization_msgs.msg import Marker
import rospy

class Ball:

	def __init__(self, loc_x, loc_y):
		self.x = loc_x
		self.y = loc_y

	def update_location(self, distance_x, distance_y):
		self.x += distance_x
		self.y += distance_y

class Paddle:

	def __init__(self, center_x, center_y):
		self.x = center_x
		self.y = center_y

	def update_location(self, distance_x):
		self.x += distance_x

class Game:

	def __init__(self, board_length, board_width, scale, ball_speed):
		self.board_width = board_width
		self.board_length = board_length 
		self.scale = scale
		self.ball_speed = ball_speed
		self.directions = ['E','W','N','S','NE','NW','SW','SE']

	def start(self):
		self.ball = Ball(self.board_width/2, self.board_length/2)
		self.computer = Paddle(self.board_width/2, 0)
		self.player = Paddle(self.board_width/2, self.board_length/2)
		self.ball_direction = choice(['N','S','NE','NW','SW','SE'])
		self.add_ball_marker()
		self.add_paddle_marker()
		self.add_score_marker()

	def add_ball_marker(self):
		pass

	def add_paddle_marker(self):
		pass

	def add_score_marker(self):
		pass

	def update_ball_marker(self):
		pass

	def update_paddle_marker(self, who):
		'''
		This method will update the scores. 

		who: 1, user won the game
		     0, computer won the game 
		'''
		pass

	def update_score(self, who):
		'''
		This method will update the scores. 

		who: 1, user won the game
		     0, computer won the game 
		'''
		pass

	def reset(self):
		self.ball.x, self.ball.y = self.board_width/2, self.board_length/2
		self.computer.x, self.computer.y = self.board_width/2, 0
		self.player.x, self.player.y = self.board_width/2, self.board_length/2
		self.update_ball_marker()
		self.update_paddle_marker(0)
		self.update_paddle_marker(1)

	def move_ball(self, time):
		'''
		This method will move the ball inside the board. It will also perform collision checking, score updates and resetting the game.

		time: it is used to update ball location in single steps and upto the speed specified. 
		'''
		if time > self.speed:
			return
		#Checking if the game is over or not
		if self.ball.y == self.scale/2 or self.ball.y == self.board_length-self.scale/2:
			#Checking if the ball reached the computer's end or not
			if self.ball.y == self.scale/2:
				self.update_score(1)
			else:
				self.update_score(0)
			self.reset()
			return
		#Checking if the ball got hit by the computer's paddle
		elif self.ball.x in range(self.computer.x-2*self.scale, self.computer.x+2.5*self.scale) and self.ball.y == 1.5*self.scale:
			if self.ball_direction == 'S':
				self.ball_direction = 'N'
				self.ball.update_location(0, self.scale)
			elif self.ball_direction == 'SE':
				self.ball_direction = 'NW'
				self.ball.update_location(-self.scale, self.scale)
			else:
				self.ball_direction = 'NE'
				self.ball.update_location(self.scale, self.scale)
		#Checking if the ball got hit by the player's paddle 
		elif self.ball.x in range(self.player.x-2*self.scale, self.player.x+2.5*self.scale) and self.ball.y == self.board_length-1.5*self.scale:
			if self.ball_direction == 'N':
				self.ball_direction = 'S'
				self.ball.update_location(0, -self.scale)
			elif self.ball_direction == 'NE':
				self.ball_direction = 'SW'
				self.ball.update_location(-self.scale, -self.scale)
			else:
				self.ball_direction = 'SE'
				self.ball.update_location(self.scale, -self.scale)
		#Checking if the ball hit the left boundary/border
		elif self.ball.x == self.scale*0.5:
			if self.ball_direction == 'SW':
				self.ball_direction = 'SE'
				self.ball.update_location(self.scale, -self.scale)
			else:
				self.ball_direction = 'NE'
				self.ball.update_location(self.scale, self.scale)
		#Checking if the ball hit the right boundary/border
		elif self.ball.x == self.board_width-scale*0.5:
			if self.ball_direction == 'NE':
				self.ball_direction = 'NW'
				self.ball.update_location(-self.scale, self.scale)
			else:
				self.ball_direction = 'SW'
				self.ball.update_location(-self.scale, -self.scale)
		#Now we are just left with the free movements of the ball
		elif self.ball_direction == 'NE':
			self.ball.update_location(self.scale, self.scale)
		elif self.ball_direction == 'N':
			self.ball.update_location(0, self.scale)
		elif self.ball_direction == 'NW':
			self.ball.update_location(-self.scale, self.scale)
		elif self.ball_direction == 'SW':
			self.ball.update_location(-self.scale, -self.scale)
		elif self.ball_direction == 'S':
			self.ball.update_location(0, -self.scale)
		else:
			self.ball.update_location(self.scale, -self.scale)
		self.update_ball_marker()
		self.move_ball(time+1)

	def move_player_paddle(self, action):
		'''
		This method will move the player's paddle
		'''
		if action == 'left' and self.player.x > 1.5*self.scale:
			self.player.update_location(-self.scale)
		elif action == 'right' and self.player.x < self.board_width-1.5*self.scale:
			self.player.update_location(self.scale)

	def move_computer_paddle(self):
		'''
		This method will move the computer's paddle
		'''
		if action == 'left' and self.computer.x > 1.5*self.scale:
			self.player.update_location(-self.scale)
		elif action == 'right' and self.computer.x < self.board_width-1.5*self.scale:
			self.player.update_location(self.scale)

if __name__ == '__main__':
	pass