#!/usr/bin/env python

import heapq
import problem 
import rospy
from std_msgs.msg import String
import argparse
from collections import deque
from copy import deepcopy

def bfs():
	init_state = problem.get_initial_state()
	possible_actions = problem.get_actions() 
	'''
	#For checking if goal state is reached:
	problem.is_goal_state() - returns 1 if true, 0 otherwise

	#For getting successor states:
	(next_state, cost) = problem.get_successor(current_state, action) - 
	next_state = State object containing x, y location and direction (if there is no next_state possible then it will have the location values as -1, -1)
	cost = cost for taking the action

	Return: list of actions for reaching goal location
	'''
	#YOUR CODE HERE

def ucs():
	init_state = problem.get_initial_state()
	possible_actions = problem.get_actions()
	''' 
	#For checking if goal state is reached:
	problem.is_goal_state() - returns 1 if true, 0 otherwise

	#For getting successor states:
	(next_state, cost) = problem.get_successor(current_state, action) - 
	next_state = State object containing x, y location and direction (if there is no next_state possible then it will have the location values as -1, -1)
	cost = cost for taking that action

	Return: list of actions for reaching goal location
	'''
	#YOUR CODE HERE

def gbfs():
	init_state = problem.get_initial_state()
	goal_state = problem.get_goal_state()
	possible_actions = problem.get_actions()
	'''
	#For checking if goal state is reached:
	problem.is_goal_state() - returns 1 if true, 0 otherwise

	#For getting successor states:
	(next_state, cost) = problem.get_successor(current_state, action) - 
	next_state = State object containing x, y location and direction (if there is no next_state possible then it will have the location values as -1, -1)
	cost = cost for taking the action

	Return: list of actions for reaching goal location
	'''
	#YOUR CODE HERE

def astar():
	init_state = problem.get_initial_state()
	goal_state = problem.get_goal_state()
	possible_actions = problem.get_actions() 
	'''
	#For checking if goal state is reached:
	problem.is_goal_state() - returns 1 if true, 0 otherwise

	#For getting successor states:
	(next_state, cost) = problem.get_successor(current_state, action) - 
	next_state = State object containing x, y location and direction (if there is no next_state possible then it will have the location values as -1, -1)
	cost = cost for taking the action

	Return: list of actions for reaching goal location
	'''
	#YOUR CODE HERE

# to execute a plan action_list = <list of actions>, use:
def exec_action_list(action_list):
	plan_str = '_'.join(action for action in action_list)
	publisher.publish(String(data = plan_str))

if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument('-a',help = 'Please mention algorithm to use. Default is BFS', metavar = 'bfs', action='store', dest='algorithm', default="bfs", type=str)
	args = parser.parse_args()
	algorithm = globals().get(args.algorithm)
	if algorithm is None:
		print 'Incorrect Algorithm name.'
		exit(1)
	rospy.init_node('search_algorithms', anonymous = True)
	publisher = rospy.Publisher('/actions',String,queue_size =10)
	actions = algorithm()
	print(actions)
	exec_action_list(actions)