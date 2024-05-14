#!/usr/bin/env python3
import rospy
from custom_msgs.srv import ExampleService

class Server:

    def __init__(self):
        self.server = rospy.Service(
            name='/example_service',
            service_class=ExampleService,
            handler=self.callback
        )

        rospy.loginfo(msg='Server is in standby')

    def callback(self, request):
        area = request.length * request.width
        perimeter = 2 * (request.length + request.width)

        rospy.loginfo(msg='A service has been requested')

        return area, perimeter

def main():
    rospy.init_node(name='server')
    Server()
    rospy.spin()

if __name__ == '__main__':
    main()