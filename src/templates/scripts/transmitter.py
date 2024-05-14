#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

class Transmitter:

    def __init__(self):
        rospy.set_param(param_name='hz', param_value=1.0)
        self.hz = rospy.get_param(param_name='hz')

        self.pub = rospy.Publisher(
            name='/example_topic',
            data_class=String,
            queue_size=10
        )

        self.publish()

    def publish(self):
        rate = rospy.Rate(hz=self.hz)

        while not rospy.is_shutdown():
            msg = String()
            msg.data = 'music'
            self.pub.publish(msg)

            rospy.loginfo(msg='Transmitting ' + msg.data)

            rate.sleep()

def main():
    rospy.init_node(name='transmitter', anonymous=False)
    Transmitter()

if __name__ == '__main__':
    main()