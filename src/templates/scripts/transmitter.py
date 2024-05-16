#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

class Transmitter:

    def __init__(self):
        # rospy.set_param(param_name='hz', param_value=1.0)
        hz = rospy.get_param(param_name='hz')

        self.pub = rospy.Publisher(name='/example_topic', data_class=String, queue_size=10)

        self.publish(hz=hz)
    
    
    def publish(self, hz):
        rate = rospy.Rate(hz=hz)

        while not rospy.is_shutdown():
            msg = String()
            msg.data = 'music'
            self.pub.publish(msg)

            rospy.loginfo(msg=f'Transmitting {msg.data}')

            rate.sleep()

if __name__ == '__main__':
    rospy.init_node(name='transmitter', anonymous=False)
    Transmitter()