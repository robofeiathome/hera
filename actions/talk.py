from translate import Translator

import actionlib
import rospy

from agent.abstract_action import AbstractAction
from agent.util.enuns import LogColor

from hera.msg import talkFeedback, talkResult, talkAction, talkGoal

from gtts_ros.msg import TalkGoal
from std_msgs.msg import String

class Talk(AbstractAction):
    """docstring for Talk."""
    def __init__(self, robot):
        super(Talk, self).__init__(robot)
        self.robot_ns = rospy.get_namespace()

        self._as = actionlib.SimpleActionServer("talk", talkAction, self.goal_callback, False)
        self._as.start()

        self.pub_vizbox_robot = rospy.Publisher('/robot_text', String, queue_size=80)

    def goal_callback(self, goal):
        result = self.execute(goal.phrase, goal.from_lang, goal.to_lang)
        self._as.set_succeeded(talkResult(result=result))

    def execute(self, phrase, from_lang="en", to_lang="en"):

        translator= Translator(from_lang=from_lang, to_lang=to_lang)
        translation = translator.translate(phrase)

        ''' Execute action talk(phrase) '''
        self.robot.add_log('Talk', translation, color=LogColor.CYAN)
        self.pub_vizbox_robot.publish(translation)

        # create goal
        goal = TalkGoal()
        # set goal
        goal.phrase = translation
        goal.lang = to_lang
        # talk
        result = self.robot.get_actuators().talk(goal)
        return 'success'
