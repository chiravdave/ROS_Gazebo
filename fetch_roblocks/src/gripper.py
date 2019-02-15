import rospy
import actionlib
from control_msgs.msg import GripperCommandAction

ACTION = 'gripper_controller/gripper_action'
CLOSE_POS = 0.0  #meters
OPEN_POS = 0.10 #meters
DEFAULT_MAX_EFFORT = 100


class FetchGripper(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient(ACTION,GripperCommandAction)
        self.client.wait_for_server()


    def open(self,max_effort=None,position=None,):
	"""
	max_effort: in Newton
	"""
        if(not position):
            position = OPEN_POS
	if(not max_effort):
            max_effort = DEFAULT_MAX_EFFORT

        self.actuateGripper(position,max_effort)

    def close(self,max_effort=None,position=None,):
	"""
	max_effort: in Newton, ensure > 35 to close
	"""
        if(not position):
            position = CLOSE_POS
	if(not max_effort):
            max_effort = DEFAULT_MAX_EFFORT

        self.actuateGripper(position,max_effort)


    def actuateGripper(self,pos,max_eff):
        goal = GripperCommandGoal()
        goal.command.position = pos
        goal.command.max_effort = DEFAULT_MAX_EFFORT
        if(max_eff):
            goal.command.max_effort = max_eff
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration(5.0))

        result = self.client.get_result()

        if(not result):
            print "ERROR: Gripper actuate result returned None, "
	else:
        return result