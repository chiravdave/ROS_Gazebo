#!/usr/bin/env python

from search.srv import *
import rospy
from gen_maze import Maze
import sys
import argparse
import time

mazeInfo = None
parser = argparse.ArgumentParser()
parser.add_argument('-d', help='for providing dimension of the grid', metavar='5', action='store', dest='grid_dimension', default=5, type=int)
parser.add_argument('-n', help='for providing no. of obstacles to be added in the grid', metavar='15', action='store', dest='n_obstacles', default=15, type=int)
parser.add_argument('-s', help='for providing random seed', metavar='32', action='store', dest='seed', default=int(time.time()), type=int)

def manhattanDistance(x1, y1, x2, y2):
	"""
	This function returns manhattan distance between two points.
	"""
	return abs(x1-x2) + abs(y1-y2)

def check_is_edge(edge):
	"""
	This function checks if two points are connected via edge or not.
	"""
	global mazeInfo
	invalid_edges = mazeInfo[1]
	if edge[0] < mazeInfo[0][0] or edge[1] < mazeInfo[0][0] or edge[2] > mazeInfo[0][1]*mazeInfo[0][3] or edge[3] > mazeInfo[0][1]*mazeInfo[0][3] or edge in invalid_edges:
			return False
	else:
		return True

def handle_get_successor(req):
	"""
		This function returns successor of a given state. 
				
		parameters:	x_cord - current x-cordinate of turtlebot           output:   x_cord - new x-cordinate of turtlebot
				    y_cord - current y-cordinate of turtlebot					  y_cord - new y-cordinate of turtlebot
				    direction - current orientation								  direction - new orientation
				    action - current action										  action_cost - cost of taking a particular action
	"""
	global mazeInfo
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
			y_cord += mazeInfo[0][3]
		elif direction == "EAST":
			x_cord += mazeInfo[0][3]
		elif direction == "SOUTH":
			y_cord -= mazeInfo[0][3]
		elif direction == "WEST":
			x_cord -= mazeInfo[0][3]
		action_cost = 1

	elif action == 'MoveB':
		if direction == "NORTH":
			y_cord -= mazeInfo[0][3]
		elif direction == "EAST":
			x_cord -= mazeInfo[0][3]
		elif direction == "SOUTH":
			y_cord += mazeInfo[0][3]
		elif direction == "WEST":
			x_cord += mazeInfo[0][3]
		action_cost = 3
	
	if x_cord < req.x or y_cord < req.y:
		isValidEdge = check_is_edge((x_cord, y_cord, req.x, req.y))
	else:
		isValidEdge = check_is_edge((req.x, req.y, x_cord, y_cord,))

	if not isValidEdge:
		return GetSuccessorResponse(-1, -1, direction, -1)

	return GetSuccessorResponse(x_cord, y_cord, direction, action_cost)
  

def handle_maze_initial_state(req):
	"""
	This function will return initial state of turtlebot3.
	"""
	global mazeInfo

	initial_state = mazeInfo[0]
	return GetInitialStateResponse(initial_state[0],initial_state[0],initial_state[2])


def handle_check_goal_state(req):
	"""
    This function will return True if turtlebot3 is at goal state otherwise it will return False.
	"""
	global mazeInfo

	goal_state = mazeInfo[0][1]*mazeInfo[0][3]

	if req.x == req.y and req.x == goal_state:
		return IsGoalStateResponse(1)

	return IsGoalStateResponse(0)

def handle_maze_goal_state(req):
	global mazeInfo
	goal_state = mazeInfo[0][1]*mazeInfo[0][3]
	return GetGoalStateResponse(goal_state,goal_state)

def handle_maze_scale(req):
	global mazeInfo
	return GetScaleResponse(mazeInfo[0][3])

def server():
    rospy.init_node('get_successor_server')
    rospy.Service('get_successor', GetSuccessor, handle_get_successor)
    rospy.Service('maze_initial_state', GetInitialState, handle_maze_initial_state)
    rospy.Service('check_goal_state', IsGoalState, handle_check_goal_state)
    rospy.Service('maze_goal_state', GetGoalState, handle_maze_goal_state)
    rospy.Service('maze_scale', GetScale, handle_maze_scale)
    print "Ready!"
    rospy.spin()

if __name__ == "__main__":
    args = parser.parse_args()
    possible_n_obstacles =  args.grid_dimension*(args.grid_dimension + 1)*2
    if args.n_obstacles > possible_n_obstacles:
    	print('Maximum no. of obstacles that could be added to the grid is {} but provided value is {}'.format(possible_n_obstacles, args.n_obstacles))
    	exit()
    my_maze = Maze()
    mazeInfo = my_maze.generate_maze(args.grid_dimension, args.n_obstacles, args.seed)
    server()