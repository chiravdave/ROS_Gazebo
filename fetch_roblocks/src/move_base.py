#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from gazebo_msgs.srv import GetModelState
from control_msgs.msg import PointHeadAction, PointHeadGoal

def get_model_pose(model_name, relative_entity_name):
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        model_pose_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        model_pose = model_pose_service(model_name, relative_entity_name)
        return model_pose
    except rospy.ServiceException, e:
        print 'Service call for getting model pose failed %s'%e
        return None

# Point the head using controller
class PointHeadClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()

    def look_at(self, target_pose, duration=1.0):
        look_goal = PointHeadGoal()
        look_goal.target.header.stamp = rospy.Time.now()
        look_goal.target.header.frame_id = 'map'
        look_goal.target.point.x = target_pose.pose.orientation.x
        look_goal.target.point.y = target_pose.pose.orientation.y
        look_goal.target.point.z = target_pose.pose.orientation.z
        look_goal.min_duration = rospy.Duration(duration)
        self.client.send_goal(look_goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr('Action server for point head not available')
        else:
            rospy.loginfo('looking at the object')

class MoveBaseClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("FetchBase: Waiting for move_base...")
        self.client.wait_for_server()

    # Moving fetch base using navigation stack
    def move_towards(self, model_name, relative_entity_name):
        '''
        model_name : name of the model in gazebo
        relative_entity_name : name of the link in the model
        '''
        target_pose = get_model_pose(model_name, relative_entity_name)
        if not target_pose:
            return None
        move_goal = MoveBaseGoal()
        move_goal.target_pose.pose.position.x = target_pose.pose.position.x - 0.5 
        move_goal.target_pose.pose.position.y = target_pose.pose.position.y - 0.5
        move_goal.target_pose.pose.position.z = target_pose.pose.position.z
        move_goal.target_pose.pose.orientation.x = target_pose.pose.orientation.x
        move_goal.target_pose.pose.orientation.y = target_pose.pose.orientation.y
        move_goal.target_pose.pose.orientation.z = target_pose.pose.orientation.z
        move_goal.target_pose.header.frame_id = 'map'
        move_goal.target_pose.header.stamp = rospy.Time.now()
        # TODO wait for things to wor
        self.client.send_goal(move_goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr('Action server for move base not available')
        else:
            rospy.loginfo('Moved at target location')
        return target_pose

if __name__ == '__main__':
    rospy.init_node('roblocks', anonymous =True)
    move_base = MoveBaseClient()
    head = PointHeadClient()
    target_pose = move_base.move_towards('table1', 'link')
    if target_pose:
        head.look_at(target_pose)