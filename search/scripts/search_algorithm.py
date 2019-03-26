#!/usr/bin/env python

import heapq
import problem 
import rospy
from std_msgs.msg import String
import argparse

def bfs():
	init_state = problem.get_initial_state()
	goal_state = problem.get_goal_state()
	possible_actions = problem.get_actions() 
	action_list = []
	#to get the next state, cost for an action on state_x use:
	(nextstate, cost) = problem.get_successor(state, action)
	'''
	YOUR CODE HERE
	'''

	return action_list

def ucs():
	init_state = problem.get_initial_state()
	goal_state = problem.get_goal_state()
	possible_actions = problem.get_actions() 
	action_list = []
	#to get the next state, cost for an action on state_x use:
	(nextstate, cost) = problem.get_successor(state, action) ## This will return state with x and y co-ordinate -1 if action is not executable.
	'''
	YOUR CODE HERE
	'''

	return action_list

def gbfs():
	init_state = problem.get_initial_state()
	goal_state = problem.get_goal_state()
	possible_actions = problem.get_actions() 
	action_list = []
	#to get the next state, cost for an action on state_x use:
	(nextstate, cost) = problem.get_successor(state, action)
	'''
	YOUR CODE HERE
	'''

	return action_list

def astar():
	init_state = problem.get_initial_state()
	goal_state = problem.get_goal_state()
	possible_actions = problem.get_actions() 
	action_list = []
	#to get the next state, cost for an action on state_x use:
	(nextstate, cost) = problem.get_successor(state, action)
	'''
	YOUR CODE HERE
	'''

	return action_list

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
	exec_action_list(actions)