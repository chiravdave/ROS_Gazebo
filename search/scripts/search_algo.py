#!/usr/bin/env python

import heapq
import problem 
import rospy
from std_msgs.msg import String
import argparse

publisher = rospy.Publisher("/actions",String,queue_size =10)
parser = argparse.ArgumentParser()
parser.add_argument('-a',help = "Please mention algorithm to use. Default is BFS", metavar = 'bfs', action='store', dest='algorithm', default="bfs", type=str)



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
    (nextstate, cost) = problem.get_successor(state, action)

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

#For astar you need to write your own heuristic function which should be both admissible and consistent
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

if __name__ == "__main__":
    args = parser.parse_args()
    algorithm = globals().get(args.algorithm)
    if algorithm is None:
        print "Incorrect Algorithm name."
        exit(1)
    actions = algorithm()
    exec_action_list(actions)