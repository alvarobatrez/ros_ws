#!/usr/bin/env python3
import rospy
import actionlib
import threading
from custom_msgs.msg import ExampleAction, ExampleResult, ExampleFeedback

class TemplateActionServer():

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

        self.goal_queue = []
        w = threading.Thread(name="worker", target=self.run_queue)
        w.start()

        rospy.loginfo(msg='Action server is in standby')

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
            goal_handle.set_canceled(result)
            rospy.logwarn(msg='Goal canceled')
        elif success:
            goal_handle.set_succeeded(result)
            rospy.loginfo(msg='Goal succeeded')
        else:
            goal_handle.set_aborted(result)
            rospy.logerr(msg='Goal aborted')

        self.cancel_goals.pop(goal_id)

    def run_queue(self):
        rate = rospy.Rate(1.0)

        while not rospy.is_shutdown():
            
            if len(self.goal_queue) > 0:
                self.execute_callback(self.goal_queue.pop(0))
            rate.sleep()

    def goal_callback(self, goal_handle):
        if goal_handle.get_goal().goal > 0:
            goal_handle.set_accepted()
            rospy.loginfo(msg='Incoming goal accepted')
        else:
            goal_handle.set_rejected()
            rospy.logwarn(msg='Incoming goal rejected')
            return
        
        self.goal_queue.append(goal_handle)
        self.cancel_goals[goal_handle.get_goal_id().id] = False

    def cancel_callback(self, goal_handle):
        goal_id = goal_handle.get_goal_id().id

        if goal_id in self.cancel_goals:
            self.cancel_goals[goal_id] = True
            rospy.logwarn(msg='Canceling queue goals')

def main():
    rospy.init_node(name="action_server")
    TemplateActionServer()
    rospy.spin()
    
if __name__ == '__main__':
    main()