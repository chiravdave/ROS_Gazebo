from numpy.random import choice

class Ball:

	def __init__(self, loc_x, loc_y):
		self.x = loc_x
		self.y = loc_y

	def update_location(self, loc_x, loc_y):
		self.x = loc_x
		self.y = loc_y

class Paddle:

	def __init__(self, distance_x, distance_y):
		self.x += distance_x
		self.y += distance_y

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

	def update_paddle_marker(self):
		pass

	def update_score(self, who):
		'''
		This method will update the scores. 

		who: if 1, then user won the game or else computer won the game 
		'''
		pass

	def move_ball(self, time):
		'''
		This method will move the ball inside the board. It will also check for all collisions, score updates and resetting the game.

		time: it is used to update ball location in single steps and upto the speed specified.  
		'''
		if time > self.speed:
			return
		if self.ball.y == self.scale or self.ball.y == self.board_length-self.scale:
			if self.ball.y == self.scale:
				self.update_score(1)
			else:
				self.update_score(0)
			self.reset()
			self.update_ball_marker()
			return
		elif self.ball.x in range(self.computer.x-1.5*self.scale, self.computer.x+2.5*self.scale) and self.ball.y == 2*self.scale:
			if self.ball_direction == 'S':
				self.ball_direction = 'N'
				self.ball.update_location(0, 1)
			elif self.ball_direction == 'SE':
				self.ball_direction = 'NW'
				self.ball.update_location(-1, 1)
			else:
				self.ball_direction = 'NE'
				self.ball.update_location(1, 1)
		elif self.ball.x in range(self.player.x-1.5*self.scale, self.player.x+2.5*self.scale) and self.ball.y == self.board_length-2*self.scale:
			if self.ball_direction == 'N':
				self.ball_direction = 'S'
				self.ball.update_location(0, -1)
			elif self.ball_direction == 'NE':
				self.ball_direction = 'SW'
				self.ball.update_location(-1, -1)
			else:
				self.ball_direction = 'SE'
				self.ball.update_location(1, -1)
		elif self.ball.x == scale:
			if self.ball_direction == 'SW':
				self.ball_direction = 'SE'
				self.ball.update_location(1, -1)
			else:
				self.ball_direction = 'NE'
				self.ball.update_location(1, 1)
		elif self.ball.x == self.board_width-scale:
			if self.ball_direction == 'NE':
				self.ball_direction = 'NW'
				self.ball.update_location(-1, 1)
			else:
				self.ball_direction = 'SW'
				self.ball.update_location(-1, -1)
		elif self.ball_direction == 'NE':
			self.ball.update_location(1, 1)
		elif self.ball_direction == 'N':
			self.ball.update_location(0, 1)
		elif self.ball_direction == 'NW':
			self.ball.update_location(-1, 1)
		elif self.ball_direction == 'SW':
			self.ball.update_location(-1, -1)
		elif self.ball_direction == 'S':
			self.ball.update_location(0, -1)
		else:
			self.ball.update_location(1, -1)
		self.update_ball_marker()
		self.move_ball(time+1)

if __name__ == '__main__':
	pass