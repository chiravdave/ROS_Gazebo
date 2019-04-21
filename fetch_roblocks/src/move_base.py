#!/usr/bin/env python

import rospy
import sys
import actionlib
from numpy import radians
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from gazebo_msgs.srv import GetModelState
from control_msgs.msg import PointHeadAction, PointHeadGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from moveit_python import MoveGroupInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from moveit_msgs.msg import MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

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
        move_goal.target_pose.pose.position.x = 0.8
        move_goal.target_pose.pose.position.y = 0
        move_goal.target_pose.pose.position.z = 0
        move_goal.target_pose.pose.orientation.x = 0
        move_goal.target_pose.pose.orientation.y = 0
        move_goal.target_pose.pose.orientation.z = 0
        move_goal.target_pose.pose.orientation.w = 1
        move_goal.target_pose.header.frame_id = 'map'
        move_goal.target_pose.header.stamp = rospy.Time.now()
        self.client.send_goal(move_goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr('Action server for move base not available')
        else:
            rospy.loginfo('Moved at target location')
        return target_pose

# Send a trajectory to controller
class FollowTrajectoryClient(object):

    def __init__(self, name, joint_names):
        self.client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % name,
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for %s..." % name)
        self.client.wait_for_server()
        self.joint_names = joint_names

    def move_to(self, positions, duration=5.0):
        if len(self.joint_names) != len(positions):
            print("Invalid trajectory position")
            return False
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = positions
        trajectory.points[0].velocities = [0.0 for _ in positions]
        trajectory.points[0].accelerations = [0.0 for _ in positions]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        self.client.send_goal(follow_goal)
        self.client.wait_for_result()

class GraspingClient(object):

    def __init__(self):
        self.arm = MoveGroupInterface('arm_with_torso', 'map')
        self.move_group = MoveGroupInterface('arm', 'base_link')

    def pick(self, target_pose):
        #print self.arm._fixed_frame, self.arm.planner_id
        #Taking coordinate values of the object 
        object_x, object_y, object_z = round(target_pose.pose.position.x, 2), round(target_pose.pose.position.y, 2), round(target_pose.pose.position.z, 2)
        print object_x, object_y, object_z  
        #Initializing offset values for translation
        offset_x, offset_y, offset_z = 0, 0, 0.05
        #Initializing offset values for rotation
        roll, pitch, yaw = 0, 0, 0
        #Applying rotation to the arm
        for i in range(10, 1, -1):
            #Adding translation along x-axis 
            final_x = object_x + offset_x
            #Adding translation along y-axis
            final_y = object_y + offset_y
            #Adding translation along z-axis
            final_z = object_z + offset_z + i*0.1
            print final_x, final_y, final_z, i*0.1
            final_quaternion = quaternion_from_euler(radians(roll), radians(pitch), radians(yaw), 'sxyz')
            pose_goal = PoseStamped()
            pose_goal.header.frame_id = 'map'
            pose_goal.pose.position.x = final_x
            pose_goal.pose.position.y = final_y
            pose_goal.pose.position.z = final_z
            pose_goal.pose.orientation.x = final_quaternion[0]
            pose_goal.pose.orientation.y = final_quaternion[1]
            pose_goal.pose.orientation.z = final_quaternion[2]
            pose_goal.pose.orientation.w = final_quaternion[3]
            pose_goal.header.stamp = rospy.Time.now()
            self.arm.moveToPose(pose_goal, 'wrist_roll_link')
            result = self.arm.get_move_action().get_result()
            if result:
                if result.error_code.val == MoveItErrorCodes.SUCCESS:
                    rospy.loginfo('Moved arm')
                    rospy.sleep(10)
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
    move_base = MoveBaseClient()
    head = PointHeadClient()
    torso_action = FollowTrajectoryClient("torso_controller", ["torso_lift_joint"])
    target_pose = move_base.move_towards('demo_cube', 'link')
    if target_pose:
        pass
        #head.look_at(target_pose)
    # Raise the torso using just a controller
    #rospy.loginfo("Raising torso...")
    #torso_action.move_to([0.4, ])
    grasp = GraspingClient()
    grasp.pick(get_model_pose('demo_cube', 'link'))