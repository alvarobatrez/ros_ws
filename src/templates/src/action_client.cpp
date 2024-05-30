#include <ros/ros.h>
#include <actionlib/client/action_client.h>
#include <custom_msgs/ExampleAction.h>

typedef actionlib::ActionClient<custom_msgs::ExampleAction>::GoalHandle GoalHandle;

class TemplateActionClient
{
    public:

    TemplateActionClient() : action_client("/example_action")
    {
        ;
    }

    void send_goal(int goal) {}

    void transition_callback() {}

    void feedback_callback() {}

    private:

    actionlib::ActionClient<custom_msgs::ExampleAction> action_client;
    GoalHandle goal_handle;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "action_client", ros::init_options::AnonymousName);
    TemplateActionClient action_client;
    ros::spin();

    return 0;
}