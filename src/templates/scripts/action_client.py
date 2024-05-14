#!/usr/bin/env python3
import rospy
import actionlib
from custom_msgs.msg import ExampleAction, ExampleGoal

class TemplateActionClient:

    def __init__(self):
        rospy.set_param(param_name='goal', param_value=10)
        goal = rospy.get_param(param_name='goal')

        self.action_client = actionlib.ActionClient(
            ns='/example_action',
            ActionSpec=ExampleAction
        )

        self.goal_handles = {}

        self.goal_handles['1'] = self.send_goal(goal=goal)

    def send_goal(self, goal):
        self.action_client.wait_for_server()

        goal_msg = ExampleGoal()
        goal_msg.goal = goal
        
        goal_handle = self.action_client.send_goal(
            goal=goal_msg,
            transition_cb=self.transition_callback,
            feedback_cb=self.feedback_callback
        )

        return goal_handle

    def transition_callback(self, goal_handle):
        # https://docs.ros.org/en/api/actionlib/html/classactionlib_1_1action__client_1_1CommState.html
        # http://docs.ros.org/en/kinetic/api/actionlib_msgs/html/msg/GoalStatus.html

        for i in self.goal_handles:
            
            if self.goal_handles[i] == goal_handle:
                break

        if goal_handle.get_comm_state() == 2:
            rospy.loginfo("Goal is active")
        
        if goal_handle.get_comm_state() == 7:
            rospy.loginfo("Goal is done")

            result = goal_handle.get_result()
            status = goal_handle.get_terminal_state()
            
            if status == 3:
                rospy.loginfo(msg=f'Goal succeeded: {result.result}')
            else:
                rospy.logerr(msg=f'Goal failed with status code: {status}')

            rospy.signal_shutdown(reason="")

    def feedback_callback(self, goal_handle, feedback):
        rospy.loginfo(msg=f'Feedback: {feedback.feedback}')

def main():
    rospy.init_node(name='action_client', anonymous=True)
    TemplateActionClient()
    rospy.spin()

if __name__ == '__main__':
    main()