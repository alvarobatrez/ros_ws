#include <ros/ros.h>
#include <std_msgs/String.h>

class Transmitter
{
    private:

    ros::NodeHandle nh;
    ros::Publisher pub;
    double hz;

    public:

    Transmitter();
    void publish();

};

Transmitter::Transmitter()
{
    nh.setParam("hz", 1.0);
    nh.getParam("hz", hz);

    pub = nh.advertise<std_msgs::String>("/example_topic", 10);

    publish();
}

void Transmitter::publish()
{
    ros::Rate rate(hz);

    while (ros::ok())
    {
        std_msgs::String msg;
        msg.data = "news";
        pub.publish(msg);

        ROS_INFO("Transmitting %s", msg.data.c_str());

        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "transmitter");
    Transmitter transmitter;
    ros::shutdown();

    return 0;
}