#!/usr/bin/env python3
import rospy
import actionlib
import threading
from custom_msgs.msg import ExampleAction, ExampleResult, ExampleFeedback

class TemplateActionServer:

    def __init__(self):
        self.action_server = actionlib.ActionServer(
            ns='/example_action',
            ActionSpec=ExampleAction,
            goal_cb=self.goal_callback,
            cancel_cb=self.cancel_callback,
            auto_start=False
        )
        self.action_server.start()

        self.cancel_goals = {}

        rospy.loginfo(msg='Action server is up')

    def execute_callback(self, goal_handle):
        rospy.loginfo(msg='Executing new goal')

        goal = goal_handle.get_goal().goal
        result = ExampleResult()
        feedback = ExampleFeedback()
        success = False
        cancel = False

        goal_id = goal_handle.get_goal_id().id

        rate = rospy.Rate(1.0)

        counter = 0
        max_num = 10

        while True:

            if self.cancel_goals[goal_id]:
                cancel = True
                break

            if counter == goal:
                success = True
                break

            if counter >= max_num:
                break

            counter += 1

            feedback.feedback = counter
            goal_handle.publish_feedback(feedback=feedback)

            rate.sleep()

        result.result = success

        if cancel:
            goal_handle.set_canceled(result=result)
            rospy.logwarn(msg='Goal canceled')
        elif success:
            goal_handle.set_succeeded(result=result)
            rospy.loginfo(msg='Goal succeeded')
        else:
            goal_handle.set_aborted(result=result)
            rospy.logerr(msg='Goal aborted')

        self.cancel_goals.pop(goal_id)

    def goal_callback(self, goal_handle):
        # Uncomment to enable a server with a single goal at the same time
        """ if len(self.cancel_goals) > 0:
            rospy.logwarn(msg='A goal is currently active. Incoming goal rejected')
            goal_handle.set_rejected()
            return """
        
        if goal_handle.get_goal().goal > 0:
            goal_handle.set_accepted()
            rospy.loginfo(msg='Incoming goal accepted')
        else:
            goal_handle.set_rejected()
            rospy.logwarn(msg='Incoming goal rejected')
            return
        
        self.cancel_goals[goal_handle.get_goal_id().id] = False

        worker = threading.Thread(name='worker', target=self.execute_callback, args=(goal_handle,))
        worker.start()

    def cancel_callback(self, goal_handle):
        goal_id = goal_handle.get_goal_id().id

        if goal_id in self.cancel_goals:
            self.cancel_goals[goal_id] = True
            rospy.logwarn(msg='Canceling goal')

if __name__ == '__main__':
    rospy.init_node(name='action_server')
    TemplateActionServer()
    rospy.spin()