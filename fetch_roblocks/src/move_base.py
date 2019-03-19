#!/usr/bin/env python

import rospy
import actionlib
from tf import TransformListener
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from gazebo_msgs.srv import GetModelState
from control_msgs.msg import PointHeadAction, PointHeadGoal
from moveit_python import MoveGroupInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from moveit_msgs.msg import MoveItErrorCodes

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
        move_goal.target_pose.pose.position.x = target_pose.pose.position.x - 0.7   
        move_goal.target_pose.pose.position.y = target_pose.pose.position.y - 0.7
        move_goal.target_pose.pose.position.z = target_pose.pose.position.z
        move_goal.target_pose.pose.orientation.x = target_pose.pose.orientation.x
        move_goal.target_pose.pose.orientation.y = target_pose.pose.orientation.y
        move_goal.target_pose.pose.orientation.z = target_pose.pose.orientation.z - 1
        move_goal.target_pose.pose.orientation.w = target_pose.pose.orientation.w
        move_goal.target_pose.header.frame_id = 'map'
        move_goal.target_pose.header.stamp = rospy.Time.now()
        self.client.send_goal(move_goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr('Action server for move base not available')
        else:
            rospy.loginfo('Moved at target location')
        return target_pose

class GraspingClient(object):
    '''
    
    '''
    def __init__(self):
        self.arm = MoveGroupInterface('arm_with_torso', 'base_link')
        self.move_group = MoveGroupInterface('arm', 'base_link')
        self.transform = TransformListener()

    def get_base_pose(self, object_pose):
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'map'
        target_pose.pose = object_pose.pose
        self.transform.waitForTransform('/map', '/base_link', rospy.Time.now(), rospy.Duration(4.0))
        base_pose = self.transform.transformPose('/base_link', target_pose)
        base_pose.pose.position.y -= 1
        base_pose.pose.position.z -= 0.3
        base_pose.pose.position.x -= 0.6 
        print(base_pose.pose.position.x, base_pose.pose.position.y)
        return base_pose

    def pick(self, object_pose):
        base_pose = self.get_base_pose(object_pose)
        #pose.pose = Pose(Point(0.042, 0.384, 1.826), Quaternion(0.173, -0.693, -0.242, 0.657))
        self.arm.moveToPose(base_pose, 'wrist_roll_link')
        result = self.arm.get_move_action().get_result()
        if result:
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo('Moved arm')
            else:
                rospy.logerr('Cannot move arm')
        else:
            rospy.logerr('No result')

    def tuck(self):
        joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(joints, pose, 0.02)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo('Tucking done')

if __name__ == '__main__':
    rospy.init_node('roblocks', anonymous =True)
    grasping = GraspingClient()
    '''move_base = MoveBaseClient()
    head = PointHeadClient()
    target_pose = move_base.move_towards('demo_cube', 'link')
    if target_pose:
        head.look_at(target_pose)'''
    grasping.pick(get_model_pose('demo_cube', 'link'))