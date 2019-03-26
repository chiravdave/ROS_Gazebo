# Table of Contents
1. [Introduction](README.md#introduction)
1. [Dependencies](README.md#dependencies)
1. [Instructions to run the code](README.md#instructions-to-run-the-code)

# Introduction
<b>Artificial Intelligence</b> is the study of building agents that act rationally. Most of the time, these agents perform some kind of search algorithm in the background in order to achieve their tasks. Typically, there are two categories of search, <b>Uninformed</b> and <b>Informed</b>. In this project, you will help your turtlebot3 to find paths through his maze world, avoiding obstacles and reaching the goal location and finally hitting the target object (and only the target object) in red.</br>

I have designed and developed this project with the motivation that it can be used to teach about <b>Search problems</b> in the course of AI.

# Dependencies
* python2.7
* numpy
* ROS
* gazebo

# Instructions to run the code

Step 1. Follow the steps mentioned at http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#ros-packages-for-gazebo to install neccessary files for the turtlebot3.

Step 2. Clone this repository and put it inside your catkin workspace.

Step 3. Go to the ./scripts folder, open the file <b>server.py</b> and update the path variable (line 10) with the absolute path of your search package. Please follow this step very carefully or else you won’t be able to view your maze in gazebo.

Step 4. Build your package and then start roscore.

Step 5. Run the ros node <b>server.py</b> as <b>rosrun search server.py</b>. You can use command line arguments to provide size of the grid, number of obstacles to add and random seed. For more details use <b>rosrun search server.py - -help</b>.

Step 6. Run the launch file <b>maze.launch</b> as <b>roslaunch search maze.launch</b>.

Step 7. Run the ros node <b>move_tbot3.py</b> as <b>rosrun search move_tbot3.py</b>.

Step 8. Implement your search algorithms in the file named <b>search_algorithm.py</b> inside the ./scripts folder. You can also run the demo ros node <b>random_walk.py</b> as <b>rosrun search random_walk.py</b> to get some idea about on how to use various helpful functions to achieve a given goal.

* Extra Information

A scale of 0.5 is used for generating maze meaning that every edge in gazebo is divided into two parts. There are few helper functions that will be required to implement your own search algorithms. All the helper functions are provided under the file named <b>problem.py</b> inside the ./scripts folder and so going through it will help a lot. <b>Please go through this information very carefully</b>. 

* <b>get_initial_state()</b> - This function returns the initial state of the turtlebot3 as a State class, which has three members (x-cordinate, y-cordinate, orientation).
* <b>is_goal_state()</b> - This function takes the state of the turtlebot3 as argument and returns 1 if it is the goal state or 0 otherwise.
* <b>get_successor()</b> - This function takes the state and the action being performed by the turtlebot3 as arguments and returns the successor states and cost associated with that action.
* <b>get_actions()</b> - This function returns list of possible actions. 