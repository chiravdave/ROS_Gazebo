# Table of Contents
1. [Introduction](README.md#introduction)
1. [Dependencies](README.md#dependencies)
1. [Instructions to run the code](README.md#instructions-to-run-the-code)
1. [Future Work](README.md#future-work)

# Introduction
Ping Pong is a <b>two-dimensional sports game</b> that simulates <b>table tennis</b>. The player controls an in-game paddle by moving it vertically across the left or right side of the screen. They can compete against another player controlling a second paddle on the opposing side. Players use the paddles to hit a ball back and forth. The goal is for each player to reach eleven points before the opponent; points are earned when one fails to return the ball to the other. </br>

I developed this game with the motivation that it can be used to teach about <b>Game Trees</b> for the course of <b>Artificial Intelligence</b>.

# Dependencies
* python2.7
* numpy
* ROS
* gazebo

# Instructions to run the code
Step 1. Clone this repository and put it inside your catkin workspace.

Step 2. Open up rviz and add a marker into it. 

Step 3. Run the ros nodes <b>test_computer_node.py</b> and <b>test_computer_node.py</b>.

Step 4. Run the ros node <b>game.py</b>

* ROS Services
   1. GameInfo.srv - Returns board length, board width, ball speed and scale (unit value by which ball and paddles are moved).

* ROS Messages
   1. BallInfo.msg - For sending current x, y location and direction of the ball.
   2. MoveComputerPaddle.msg - For sending direction to move computer's paddle.
   2. MovePlayerPaddle.msg - For sending direction to move player's paddle.
   
* Extra Information

I have provided two example files named <b>test_computer_node.py</b> and <b>test_player_node.py</b> for the reference. The game begins with the ball moving in some random direction and with some defined speed (1). The game updates the ball location at every 1/3 sec. The locations of the ball and paddles are updated depending on the <b>scale value (0.5)</b>. There are three possible actions for the paddle movement <b>(stay, up and down)</b>. The player/computer make a move only when the game publishes (BallInfo.msg) the new information of the ball.

Implement your own logic for player nodes and either play with the computer or against each other.

# Future Work
For future work, I'll be adding dynamics to the environment such that the ball's movement are not linear.
