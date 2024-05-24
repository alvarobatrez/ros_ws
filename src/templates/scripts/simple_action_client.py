#!/usr/bin/env python3
import rospy
import actionlib
from custom_msgs.msg import ExampleAction, ExampleGoal

class TemplateActionClient:

    def __init__(self):
        goal = rospy.get_param(param_name='goal')

        self.action_client = actionlib.SimpleActionClient(ns='/example_action', ActionSpec=ExampleAction)

        self.send_goal(goal=goal)
        self.action_client.wait_for_result()

    def send_goal(self, goal):
        self.action_client.wait_for_server()

        goal_msg = ExampleGoal()
        goal_msg.goal = goal

        self.action_client.send_goal(
            goal=goal_msg,
            done_cb=self.done_callback,
            feedback_cb=self.feedback_callback
        )

    def done_callback(self, status, result):
        goal_id = {
            0 : 'PENDING',
            1 : 'ACTIVE',
            2 : 'CANCELED',
            3 : 'SUCCEEDED',
            4 : 'ABORTED',
            5 : 'REJECTED',
            6 : 'CANCELING',
            7 : 'RECALLING',
            8 : 'RECALLED',
            9 : 'LOST'
        }
        
        if status == 3:
            rospy.loginfo(msg=f'Goal succeeded: {result.result}')
        else:
            rospy.logerr(msg=f'Goal failed with status: {goal_id[status]}')

    def feedback_callback(self, feedback):
        rospy.loginfo(msg=f'Feedback: {feedback.feedback}')

if __name__ == '__main__':
    rospy.init_node(name='action_client')
    TemplateActionClient()