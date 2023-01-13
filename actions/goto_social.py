import actionlib
import rospy
import tf

from agent.abstract_action import AbstractAction
from agent.util.enuns import LogColor

from hera.msg import gotosocialFeedback, gotosocialResult, gotosocialAction, gotosocialGoal

from social_move_base.msg import SocialMoveBaseGoal

class GotoSocial(AbstractAction):
    """docstring for GotoSocila."""
    def __init__(self, robot):
        super(GotoSocial, self).__init__(robot)
        self.robot_ns = rospy.get_namespace()

        self._as = actionlib.SimpleActionServer("gotosocial", gotosocialAction, self.goal_callback, False)
        self._as.start()

        self.tf_listener = tf.TransformListener()

    def goal_callback(self, goal):
        result = self.execute(goal.location)
        self._as.set_succeeded(gotosocialResult(result=result))

    def execute(self, location):
        self.robot.add_log('GotoSocial', location, color=LogColor.YELLOW)
    
        # create goal
        goal = SocialMoveBaseGoal()
        # goal.header.frame_id = 'map'
        # goal.header.stamp = rospy.Time.now()
        goal.target_name = location
        goal.navigation_type = 'LOCAL'
    
        return self.robot.get_actuators().gotosocial(goal)

