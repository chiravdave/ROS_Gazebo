class Ball:

	def __init__(self, center_x, center_y, direction):
		self.x = center_x
		self.y = center_y
		self.direction = direction

	def update_location(self, distance_x, distance_y):
		self.x = round(self.x + distance_x, 1)
		self.y = round(self.y + distance_y, 1)

	def update_direction(self, direction):
		self.direction = direction

	def __str__(self):
		return 'ball is at loc ({}, {})'.format(self.x, self.y)

	def __repr__(self):
		return 'ball is at loc ({}, {})'.format(self.x, self.y)