#include <ros/ros.h>
#include <custom_msgs/ExampleService.h>

class Client
{
    public:

    Client()
    {
        nh.getParam("length", length);
        nh.getParam("width", width);
        
        client = nh.serviceClient<custom_msgs::ExampleService>("/example_service");

        Client::call(length, width);
    }

    private:

    void call(double length, double width)
    {
        client.waitForExistence();
        
        srv.request.length = length;
        srv.request.width = width;

        if (client.call(srv))
        {
            ROS_INFO("Area = %f", srv.response.area);
            ROS_INFO("Perimeter = %f", srv.response.perimeter);
        }
        else
        {
            ROS_ERROR("Service call failed");
        }
    }

    ros::NodeHandle nh;
    ros::ServiceClient client;
    custom_msgs::ExampleService srv;
    double length, width;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "client");
    Client client;

    return 0;
}