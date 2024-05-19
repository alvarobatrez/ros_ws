#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

class Receiver:

    def __init__(self):
        self.sub = rospy.Subscriber(name='/example_topic', data_class=String, callback=self.callback, queue_size=10)

    def callback(self, msg):
        rospy.loginfo(msg=f'Receiving {msg.data}')
        

if __name__ == '__main__':
    rospy.init_node(name='receiver')
    Receiver()
    rospy.spin()