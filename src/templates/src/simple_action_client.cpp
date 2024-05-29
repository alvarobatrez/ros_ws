#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <custom_msgs/ExampleAction.h>

class TemplateActionClient
{
    public:

    TemplateActionClient() : action_client("/example_action", true)
    {
        nh.getParam("goal", goal);

        TemplateActionClient::send_goal(goal);        
        action_client.waitForResult();
    }

    void send_goal(int goal)
    {
        action_client.waitForServer();

        custom_msgs::ExampleGoal goal_msg;
        goal_msg.goal = goal;

        action_client.sendGoal
        (
            goal_msg,
            boost::bind(&TemplateActionClient::done_callback, this, _1, _2),
            boost::bind(&TemplateActionClient::active_callback, this),
            boost::bind(&TemplateActionClient::feedback_callback, this, _1)
        );
    }

    void done_callback
    (
        const actionlib::SimpleClientGoalState &state,
        const custom_msgs::ExampleResultConstPtr &result
    )
    {
        if (action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            std::string res = (result->result == goal) ? "true" : "false";
            ROS_INFO("Goal succeeded: %s", res.c_str());
        }
        else
        {
            ROS_ERROR("Goal failed with status: %s", action_client.getState().toString().c_str());
        }
    }

    void active_callback() {}

    void feedback_callback(const custom_msgs::ExampleFeedbackConstPtr &feedback)
    {
        ROS_INFO("Feedback: %i", (int)feedback->feedback);
    }

    private:

    ros::NodeHandle nh;
    actionlib::SimpleActionClient<custom_msgs::ExampleAction> action_client;
    int goal;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "action_client");
    TemplateActionClient action_client;

    return 0;
}