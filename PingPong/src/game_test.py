#!/usr/bin/env python

from numpy.random import choice
import argparse

class Ball:

	def __init__(self, center_x, center_y):
		self.x = center_x
		self.y = center_y

	def update_location(self, distance_x, distance_y):
		print('change value by {} and {}'.format(distance_x, distance_y))
		self.x = round(self.x + distance_x, 1)
		self.y = round(self.y + distance_y, 1)

	def __str__(self):
		return 'ball is at loc ({}, {})'.format(self.x, self.y)

	def __repr__(self):
		return 'ball is at loc ({}, {})'.format(self.x, self.y)

class Paddle:

	def __init__(self, center_x, center_y):
		self.x = center_x
		self.y = center_y

	def update_location(self, distance_x):
		print('change value by {}'.format(distance_x))
		self.x = round(self.x + distance_x, 1)

	def __str__(self):
		return 'paddle is at loc({}, {})'.format(self.x, self.y)

	def __repr__(self):
		return 'paddle is at loc({}, {})'.format(self.x, self.y)

class Game:

	def __init__(self, board_length, board_width, ball_speed, scale=0.2):
		self.board_width = float(board_width)
		self.board_length = float(board_length)
		self.scale = scale
		self.ball_speed = ball_speed
		self.directions = ['E','W','N','S','NE','NW','SW','SE']

	def start(self):
		if self.board_length % 2 == 0:
			self.ball = Ball(self.board_width/2, self.board_length/2+self.scale/2)
		else:
			self.ball = Ball(self.board_width/2, self.board_length/2)
		self.computer = Paddle(self.board_width/2, 0)
		self.player = Paddle(self.board_width/2, self.board_length)
		self.ball_direction = choice(['N','S','NE','NW','SW','SE'])

	def value_in_range(self, value, low, high):
		'''
		This method will check if a value is in the range of low and high which could be float
		'''
		if low <= value and value <= high:
			return True
		else:
			return False 

	def move_ball(self, time):
		'''
		This method will move the ball inside the board. It will also perform collision checking, score updates and resetting the game.

		time: it is used to update ball location in single steps and upto the speed specified. 
		'''
		if time > self.ball_speed:
			return
		#Checking if the game is over or not
		elif self.ball.y == self.scale/2 or self.ball.y == self.board_length-self.scale/2:
			#Checking if the ball reached the computer's end or not
			if self.ball.y == self.scale/2:
				print('Player won!')
			else:
				print('Computer won!')
			self.start()
			return
		#Checking if the ball got hit by the computer's paddle
		elif self.value_in_range(self.ball.x, self.computer.x-2*self.scale, self.computer.x+2*self.scale) and self.ball.y == round(1.5*self.scale, 1):
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
		elif self.value_in_range(self.ball.x, self.player.x-2*self.scale, self.player.x+2*self.scale) and self.ball.y == round(self.board_length-1.5*self.scale, 1):
			if self.ball_direction == 'N':
				self.ball_direction = 'S'
				self.ball.update_location(0, -self.scale)
			elif self.ball_direction == 'NE':
				self.ball_direction = 'SW'
				self.ball.update_location(-self.scale, -self.scale)
			else:
				self.ball_direction = 'SE'
				self.ball.update_location(self.scale, -self.scale)
		#Checking if the ball hit the top boundary/border
		elif self.ball.x == self.scale*0.5:
			if self.ball_direction == 'SW':
				self.ball_direction = 'SE'
				self.ball.update_location(self.scale, -self.scale)
			else:
				self.ball_direction = 'NE'
				self.ball.update_location(self.scale, self.scale)
		#Checking if the ball hit the bottom boundary/border
		elif self.ball.x == self.board_width-self.scale*0.5:
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
		self.move_ball(time+1)

	def move_player_paddle(self, data):
		'''
		This method will move the player's paddle
		'''
		if data == 'up' and self.player.x > 1.5*self.scale:
			self.player.update_location(-self.scale)
		elif data == 'down' and self.player.x < self.board_width-1.5*self.scale:
			self.player.update_location(self.scale)

	def move_computer_paddle(self, data):
		'''
		This method will move the computer's paddle
		'''
		if data == 'up' and self.computer.x > 1.5*self.scale:
			self.computer.update_location(-self.scale)
		elif data == 'down' and self.computer.x < self.board_width-1.5*self.scale:
			self.computer.update_location(self.scale)

if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument('-w', help='for providing board width', metavar='3', action='store', dest='board_width', default=3, type=int)
	parser.add_argument('-l', help='for providing board length', metavar='4', action='store', dest='board_length', default=4, type=int)
	parser.add_argument('-s', help='for providing ball speed', metavar='1', action='store', dest='ball_speed', default=1, type=int)
	args = parser.parse_args()
	game = Game(args.board_length, args.board_width, args.ball_speed)
	game.start()
	print(game.ball_direction)
	print("Enter 1 to move ball, 2 to move computer's paddle, 3 to move player's paddle and 4 to exit")
	while True:
		what = int(input("Enter your choice"))
		if what == 1:
			game.move_ball(1)
			print(game.ball, game.ball_direction)
		elif what == 2:
			action = input("which direction to move")
			game.move_computer_paddle(action)
			print(game.computer)
		elif what == 3:
			action = input("which direction to move")
			game.move_player_paddle(action)
			print(game.player)
		elif what == 4:
			exit()