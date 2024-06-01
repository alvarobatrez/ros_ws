#include <ros/ros.h>
#include <actionlib/client/action_client.h>
#include <custom_msgs/ExampleAction.h>

typedef actionlib::ClientGoalHandle<custom_msgs::ExampleAction> GoalHandle;

class TemplateActionClient
{
    public:

    std::map<int, GoalHandle*> goal_handles;

    TemplateActionClient() : action_client("/example_action")
    {
        action_client.waitForActionServerToStart();
    }

    void transition_callback(const GoalHandle goal_handle)
    {        
        int index = 0;
        std::map<int, GoalHandle*>::iterator i = goal_handles.begin();
        while (i != goal_handles.end())
        {
            if (*(i->second) == goal_handle)
            {
                index = i->first;
                break;
            }
            i++;
        }

        actionlib::CommState CommState = goal_handle.getCommState();
        
        if (CommState.toString() == "ACTIVE")
        {
            ROS_INFO("Goal is active");
        }

        if (CommState.toString() == "DONE")
        {
            ROS_INFO("Goal is done");

            custom_msgs::ExampleResultConstPtr result = goal_handle.getResult();
            actionlib::TerminalState status = goal_handle.getTerminalState();

            if (status.toString() == "SUCCEEDED")
            {
                std::string res = (result->result == goal_msg.goal) ? "true" : "false";
                ROS_INFO("Goal succeeded: %s", res.c_str());
            }
            else
            {
                ROS_ERROR("Goal failed with status: %s", status.toString().c_str());
            }

            ros::shutdown();
        }
    }

    void feedback_callback
    (
        const GoalHandle goal_handle,
        const custom_msgs::ExampleFeedbackConstPtr &feedback
    )
    {
        ROS_INFO("Feedback: %i", (int)feedback->feedback);
    }

    GoalHandle send_goal(int goal)
    {
        goal_msg.goal = goal;
        GoalHandle goal_handle = action_client.sendGoal
        (
            goal_msg,
            boost::bind(&TemplateActionClient::transition_callback, this, _1),
            boost::bind(&TemplateActionClient::feedback_callback, this, _1, _2)
        );
        
        return goal_handle;
    }

    private:

    actionlib::ActionClient<custom_msgs::ExampleAction> action_client;
    custom_msgs::ExampleGoal goal_msg;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "action_client");
    
    ros::AsyncSpinner spinner(5);
    spinner.start();

    TemplateActionClient action_client;

    int goal;
    ros::param::get("goal", goal);

    GoalHandle goal_handle = action_client.send_goal(goal);
    action_client.goal_handles[1] = &goal_handle;
    
    ros::waitForShutdown();

    return 0;
}