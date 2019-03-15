#!/usr/bin/env python

from numpy.random import choice
from visualization_msgs.msg import Marker
import rospy

class Ball:

	def __init__(self, center_x, center_y):
		self.x = center_x
		self.y = center_y

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
		self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=100)

	def start(self):
		self.ball = Ball(self.board_width/2, self.board_length/2)
		self.computer = Paddle(self.board_width/2, 0)
		self.player = Paddle(self.board_width/2, self.board_length/2)
		self.ball_direction = choice(['N','S','NE','NW','SW','SE'])
		self.create_board_rviz()

	def create_board_rviz(self):
		'''
		This method will create the board in rviz
		'''
		boundary_info = [(-0.1, self.board_length/2, 0.2, self.board_length), (self.board_width/2, self.board_length+0.1, self.board_width, 0.2),
						(self.board_width+0.1, self.board_length/2, 0.2, self.board_length), (self.board_width/2, -0.1, self.board_width, 0.2)]
		self.add_game_boundary_rviz(boundary_info)
		self.add_ball_marker(self.board_width/2, self.board_length/2)
		#Adding marker for computer's paddle
		self.add_paddle_marker(self.board_width/2, self.scale/2, 0)
		#Adding marker for player's paddle
		self.add_paddle_marker(self.board_width/2, self.board_length-self.scale/2, 1)
		self.add_score_marker()

	def add_game_boundary_rviz(self, boundary_info):
		'''
		This method will add marker for the boundaries in the board game in rviz

		boundary_info: [(center_x, center_y, scale_x, scale_y)] is a list of tuple containing values for all the boundaries starting from top and going in clockwise direction.
		'''
		unique_ns = ['top', 'right', 'bottom', 'left']
		index = 0
		for boundary in boundary_info:
			boundary_marker = Marker()
			boundary_marker.header.frame_id = "/map"
			boundary_marker.header.stamp = rospy.Time.now()
			boundary_marker.action = boundary_marker.ADD
			boundary_marker.type = boundary_marker.CUBE
			boundary_marker.ns = unique_ns[index]
			boundary_marker.id = index+4
			boundary_marker.pose.position.x = boundary[0]
			boundary_marker.pose.position.y = boundary[1]
			boundary_marker.pose.position.z = 0.025
			boundary_marker.pose.orientation.w = 0
			boundary_marker.pose.orientation.x = 0
			boundary_marker.pose.orientation.y = 0
			boundary_marker.pose.orientation.z = 0
			boundary_marker.scale.x = boundary[2]
			boundary_marker.scale.y = boundary[3]
			boundary_marker.scale.z = 0.05
			boundary_marker.color.a = 1
			boundary_marker.color.r = 1
			boundary_marker.color.g = 1
			boundary_marker.color.b = 1 
			self.marker_publisher.publish(boundary_marker)
			index += 1

	def add_ball_marker(self, center_x, center_y):
		'''
		This method will add a marker for the ball
		'''
		self.ball_marker = Marker()
		self.ball_marker.header.frame_id = "/map"
		self.ball_marker.header.stamp = rospy.Time.now()
		self.ball_marker.action = self.ball_marker.ADD
		self.ball_marker.type = self.ball_marker.CYLINDER
		self.ball_marker.ns = 'ball'
		self.ball_marker.id = 1
		self.ball_marker.scale.x = self.scale
		self.ball_marker.scale.y = self.scale
		self.ball_marker.scale.z = 0.05
		self.ball_marker.color.a = 1
		self.ball_marker.color.r = 1
		self.ball_marker.color.g = 1
		self.ball_marker.color.b = 1
		self.ball_marker.pose.orientation.w = 0
		self.ball_marker.pose.orientation.x = 0
		self.ball_marker.pose.orientation.y = 0
		self.ball_marker.pose.orientation.z = 0
		self.ball_marker.pose.position.x = center_x
		self.ball_marker.pose.position.y = center_y
		self.ball_marker.pose.position.z = 0.025 
		self.marker_publisher.publish(self.ball_marker)

	def add_paddle_marker(self, center_x, center_y, who):
		'''
		This method will add marker for the paddles. 

		who: 1, for user paddle
		     0, for computer 
		'''
		marker = Marker()
		marker.header.frame_id = "/map"
		marker.header.stamp = rospy.Time.now()
		marker.action = marker.ADD
		marker.type = marker.CUBE
		marker.scale.x = 3*self.scale
		marker.scale.y = self.scale/2
		marker.scale.z = 0.05
		marker.pose.orientation.w = 0
		marker.pose.orientation.x = 0
		marker.pose.orientation.y = 0
		marker.pose.orientation.z = 0
		marker.pose.position.x = center_x
		marker.pose.position.z = 0.025
		if who == 0:
			self.computer_marker = marker
			self.computer_marker.ns = 'computer'
			self.computer_marker.id = 2
			self.computer_marker.color.a = 1
			self.computer_marker.color.r = 1
			self.computer_marker.color.g = 1
			self.computer_marker.color.b = 1
			self.computer_marker.pose.position.y = center_y
		else:
			self.player_marker = marker
			self.player_marker.ns = 'player'
			self.player_marker.id = 3
			self.player_marker.color.a = 1
			self.player_marker.color.r = 0
			self.player_marker.color.g = 0
			self.player_marker.color.b = 1
			self.player_marker.pose.position.y = center_y
		self.marker_publisher.publish(marker)

	def add_score_marker(self):
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
	rospy.init_node('game', anonymous=True)
	game = Game(2, 2, 0.2, 1)
	i = 1
	while i <= 2:
		game.create_board_rviz()
		rospy.sleep(2)
		i += 1