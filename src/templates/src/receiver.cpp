#include <ros/ros.h>
#include <std_msgs/String.h>

class Receiver
{
    public:

    Receiver()
    {
        sub = nh.subscribe("/example_topic", 10, &Receiver::callback, this);
    }

    private:

    void callback(const std_msgs::String::ConstPtr &msg)
    {
        ROS_INFO("Receiving %s", msg->data.c_str());
    }

    ros::NodeHandle nh;
    ros::Subscriber sub;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "receiver");
    Receiver receiver;
    ros::spin();
    
    return 0;
}