#!/usr/bin/env python3
import rospy
import actionlib
from custom_msgs.msg import ExampleAction, ExampleResult, ExampleFeedback

class TemplateActionServer():

    def __init__(self):
        self.action_server = actionlib.SimpleActionServer(
            name='/example_action',
            ActionSpec=ExampleAction,
            execute_cb=self.execute_callback,
            auto_start=False
        )

        self.action_server.start()

        rospy.loginfo(msg='Action server is in standby')

    def execute_callback(self, goal_handle):
        rospy.loginfo(msg='Executing new goal')

        goal = goal_handle.goal
        result = ExampleResult()
        feedback = ExampleFeedback()
        success = False
        cancel = False

        rate = rospy.Rate(1.0)

        counter = 0
        max_num = 10

        while True:

            if self.action_server.is_preempt_requested():
                cancel = True
                break

            if counter == goal:
                success = True
                break

            if counter >= max_num:
                break

            counter += 1

            feedback.feedback = counter
            self.action_server.publish_feedback(feedback=feedback)

            rate.sleep()

        result.result = success

        if cancel:
            self.action_server.set_preempted(result)
            rospy.logwarn(msg='Goal canceled')
        elif success:
            self.action_server.set_succeeded(result)
            rospy.loginfo(msg='Goal succeeded')
        else:
            self.action_server.set_aborted(result)
            rospy.logerr(msg='Goal aborted')
            
def main():
    rospy.init_node(name="action_server")
    TemplateActionServer()
    rospy.spin()
    
if __name__ == '__main__':
    main()