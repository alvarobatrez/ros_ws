#!/usr/bin/env python3
import rospy
from custom_msgs.srv import ExampleService

class Client:

    def __init__(self):
        rospy.set_param(param_name='length', param_value=1.0)
        rospy.set_param(param_name='width', param_value=1.0)
        length = rospy.get_param(param_name='length')
        width = rospy.get_param(param_name='width')

        self.client = rospy.ServiceProxy(
            name='/example_service',
            service_class=ExampleService
        )

        self.call(length=length, width=width)

    def call(self, length, width):
        rospy.wait_for_service(service='/example_service')

        try:
            response = self.client(length, width)
        except rospy.ServiceException as e:
            rospy.logerr(msg='Service call failed: ' + str(e))
        else:
            rospy.loginfo(f'Area = {response.area}')
            rospy.loginfo(f'Perimeter = {response.perimeter}')

def main():
    rospy.init_node(name='client')
    Client()

if __name__ == '__main__':
    main()