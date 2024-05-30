#include <ros/ros.h>
#include <actionlib/client/action_client.h>
#include <custom_msgs/ExampleAction.h>

typedef actionlib::ActionClient<custom_msgs::ExampleAction>::GoalHandle GoalHandle;

class TemplateActionClient
{
    public:

    TemplateActionClient() : action_client("/example_action")
    {
        ros::param::get("goal", goal);
    }

    GoalHandle send_goal(int goal)
    {
        action_client.waitForActionServerToStart();

        custom_msgs::ExampleGoal goal_msg;
        goal_msg.goal = goal;

        goal_handle = action_client.sendGoal
        (
            goal_msg,
            boost::bind(&TemplateActionClient::transition_callback, this, _1),
            boost::bind(&TemplateActionClient::feedback_callback, this, _1, _2)
        );

        return goal_handle;
    }

    void transition_callback(const GoalHandle goal_handle)
    {
        std::map<int, std::string> goal_id = {
            {0, "PENDING"},
            {1, "ACTIVE"},
            {2, "CANCELED"},
            {3, "SUCCEEDED"},
            {4, "ABORTED"},
            {5, "REJECTED"},
            {6, "CANCELING"},
            {7, "RECALLING"},
            {8, "RECALLED"},
            {9, "LOST"}
        };

        actionlib::CommState state = goal_handle.getCommState();

        if (state.toString() == "ACTIVE")
        {
            ROS_INFO("Goal is active");
        }

        if (state.toString() == "DONE")
        {
            ROS_INFO("Goal is done");

            custom_msgs::ExampleResultConstPtr result = goal_handle.getResult();
            actionlib::TerminalState status = goal_handle.getTerminalState();

            if (status.toString() == "SUCCEEDED")
            {
                std::string res = (result->result == goal) ? "true" : "false";
                ROS_INFO("Goal succeeded: %s", res.c_str());
            }
            else
            {
                ROS_ERROR("Goal failed with status: %s", status.toString().c_str());
            }

            ros::waitForShutdown();
        }
    }

    void feedback_callback
    (
        const GoalHandle goal_handle,
        const custom_msgs::ExampleFeedbackConstPtr &feedback
    )
    {
        ;
    }

    private:

    actionlib::ActionClient<custom_msgs::ExampleAction> action_client;
    GoalHandle goal_handle;
    int goal;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "action_client", ros::init_options::AnonymousName);
    TemplateActionClient action_client;
    ros::spin();

    return 0;
}