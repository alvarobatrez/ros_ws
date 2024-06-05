#include <ros/ros.h>
#include <std_msgs/String.h>

class Transmitter
{
    public:

    Transmitter()
    {
        // nh.setParam("hz", 1.0);
        nh.getParam("hz", hz);
        pub = nh.advertise<std_msgs::String>("/example_topic", 10);
        publish(hz);
    }

    private:

    void publish(double)
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

    ros::NodeHandle nh;
    ros::Publisher pub;
    double hz;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "transmitter", ros::init_options::AnonymousName);
    Transmitter transmitter;

    return 0;
}