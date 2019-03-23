class Paddle:

	def __init__(self, center_x, center_y):
		self.x = center_x
		self.y = center_y

	def update_location(self, distance_x):
		self.x = round(self.x + distance_x, 1)

	def __str__(self):
		return 'paddle is at loc({}, {})'.format(self.x, self.y)

	def __repr__(self):
		return 'paddle is at loc({}, {})'.format(self.x, self.y)