#!/usr/bin/env python3
import rospy
from custom_msgs.srv import ExampleService

class Server:

    def __init__(self):
        rospy.loginfo(msg='hola mundo')
        

if __name__ == '__main__':
    rospy.init_node(name='server')
    Server()