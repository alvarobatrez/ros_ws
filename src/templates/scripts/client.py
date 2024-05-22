#!/usr/bin/env python3
import rospy
from custom_msgs.srv import ExampleService

class Client:

    def __init__(self):
        length = rospy.get_param(param_name='length')
        width = rospy.get_param(param_name='width')

        self.client = rospy.ServiceProxy(name='/example_service', service_class=ExampleService)

        self.call(length=length, width=width)
    
    def call(self, length, width):
        rospy.wait_for_service(service='/example_service')

        try:
            response = self.client(length, width)
        except rospy.ServiceException as e:
            rospy.logerr(msg=f'Service call failed: {e}')
        else:
            rospy.loginfo(msg=f'Area = {response.area}')
            rospy.loginfo(msg=f'Perimeter = {response.perimeter}')        

if __name__ == '__main__':
    rospy.init_node(name='client')
    Client()