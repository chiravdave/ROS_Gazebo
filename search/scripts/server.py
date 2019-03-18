#!/usr/bin/env python

from search.srv import *
import rospy
from gen_maze import Maze
import sys
import argparse
import time

path = '/home/local/ASUAD/cdave1/catkin_ws/src/search'

class Server:

	def __init__(self, grid_dimension, n_obstacles, seed):
		self.maze = Maze(grid_dimension)
		self.maze.generate_maze(n_obstacles, seed, path)

	def start_server(self):
		rospy.init_node('get_successor_server', anonymous = True)
		rospy.Service('get_successor', GetSuccessor, self.handle_get_successor)
		rospy.Service('maze_initial_state', GetInitialState, self.handle_maze_initial_state)
		rospy.Service('check_goal_state', IsGoalState, self.handle_check_goal_state)
		rospy.Service('maze_goal_state', GetGoalState, self.handle_maze_goal_state)
		print "Ready!"
		rospy.spin()

	def manhattanDistance(self, x1, y1, x2, y2):
		"""
		This function returns manhattan distance between two points.
		"""
		return abs(x1-x2) + abs(y1-y2)

	def check_is_edge(self, edge):
		"""
		This function checks if two points are connected via edge or not.
		"""
		if edge[0] < self.maze.start or edge[1] < self.maze.start or edge[2] > self.maze.end*0.5 or edge[3] > self.maze.end*0.5 or edge in self.maze.blocked_edges:
				return False
		else:
			return True

	def handle_get_successor(self, req):
		"""
			This function returns successor of a given state. 
					
			parameters:	x_cord - current x-cordinate of turtlebot           output:   x_cord - new x-cordinate of turtlebot
					    y_cord - current y-cordinate of turtlebot					  y_cord - new y-cordinate of turtlebot
					    direction - current orientation								  direction - new orientation
					    action - current action										  action_cost - cost of taking a particular action
		"""
		directionList = ["NORTH", "EAST","SOUTH","WEST"]
		x_cord, y_cord, direction, action = req.x, req.y, req.direction, req.action

		#Checking requested action and making changes in states
		if action == 'TurnCW':
			index = directionList.index(req.direction)
			direction = directionList[(index+1)%4]
			action_cost = 2

		elif action == 'TurnCCW':
			index = directionList.index(req.direction)
			direction = directionList[(index-1)%4]
			action_cost = 2

		elif action == 'MoveF':
			if direction == "NORTH":
				y_cord += 0.5
			elif direction == "EAST":
				x_cord += 0.5
			elif direction == "SOUTH":
				y_cord -= 0.5
			elif direction == "WEST":
				x_cord -= 0.5
			action_cost = 1

		elif action == 'MoveB':
			if direction == "NORTH":
				y_cord -= 0.5
			elif direction == "EAST":
				x_cord -= 0.5
			elif direction == "SOUTH":
				y_cord += 0.5
			elif direction == "WEST":
				x_cord += 0.5
			action_cost = 3
		
		if x_cord < req.x or y_cord < req.y:
			isValidEdge = self.check_is_edge((x_cord, y_cord, req.x, req.y))
		else:
			isValidEdge = self.check_is_edge((req.x, req.y, x_cord, y_cord,))

		if not isValidEdge:
			return GetSuccessorResponse(-1, -1, direction, -1)

		return GetSuccessorResponse(x_cord, y_cord, direction, action_cost)

	def handle_maze_initial_state(self, req):
		"""
		This function will return initial state of turtlebot3.
		"""
		return GetInitialStateResponse(self.maze.start, self.maze.start, 'EAST')

	def handle_check_goal_state(self, req):
		"""
		This function will return True if turtlebot3 is at goal state otherwise it will return False.
		"""
		if req.x == req.y and req.x == self.maze.end*0.5:
			return IsGoalStateResponse(1)

		return IsGoalStateResponse(0)

	def handle_maze_goal_state(req):
		goal_state = self.maze.end*0.5
		return GetGoalStateResponse(goal_state, goal_state)

if __name__ == "__main__":
	parser = argparse.ArgumentParser()
	parser.add_argument('-d', help='for providing dimension of the grid', metavar='5', action='store', dest='grid_dimension', default=5, type=int)
	parser.add_argument('-n', help='for providing no. of obstacles to be added in the grid', metavar='15', action='store', dest='n_obstacles', default=15, type=int)
	parser.add_argument('-s', help='for providing random seed', metavar='32', action='store', dest='seed', default=int(time.time()), type=int)
	args = parser.parse_args()
	possible_n_obstacles =  args.grid_dimension*(args.grid_dimension + 1)*2
	if args.n_obstacles > possible_n_obstacles:
		print('Maximum no. of obstacles that could be added to the grid is {} but provided value is {}'.format(possible_n_obstacles, args.n_obstacles))
		exit()
	my_server = Server(args.grid_dimension, args.n_obstacles, args.seed)
	my_server.start_server()