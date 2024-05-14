#!/usr/bin/env python3
import rospy
import actionlib
from custom_msgs.msg import ExampleAction, ExampleGoal

class TemplateActionClient:

    def __init__(self):
        rospy.set_param(param_name='goal', param_value=10)
        goal = rospy.get_param(param_name='goal')

        self.action_client = actionlib.SimpleActionClient(
            ns='/example_action',
            ActionSpec=ExampleAction
        )

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
        # http://docs.ros.org/en/kinetic/api/actionlib_msgs/html/msg/GoalStatus.html
        
        if status == 3:
            rospy.loginfo(msg=f'Goal succeeded: {result.result}')
        else:
            rospy.logerr(msg=f'Goal failed with status code: {status}')

    def feedback_callback(self, feedback):
        rospy.loginfo(msg=f'Feedback: {feedback.feedback}')

def main():
    rospy.init_node(name='action_client')
    TemplateActionClient()

if __name__ == '__main__':
    main()