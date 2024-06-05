#include <ros/ros.h>
#include <custom_msgs/ExampleService.h>

class Server
{
    public:

    Server()
    {
        server = nh.advertiseService("/example_service", &Server::callback, this);

        ROS_INFO("The server is up");
    }

    private:

    bool callback(custom_msgs::ExampleService::Request &req, custom_msgs::ExampleService::Response &res)
    {
        res.area = req.length * req.width;
        res.perimeter = 2 * (req.length + req.width);

        ROS_INFO("A service has been called");

        return true;
    }

    ros::NodeHandle nh;
    ros::ServiceServer server;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "server");
    Server server;
    ros::spin();
    
    return 0;
}