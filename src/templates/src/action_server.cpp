#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <custom_msgs/ExampleAction.h>
#include <thread>

typedef actionlib::ActionServer<custom_msgs::ExampleAction>::GoalHandle GoalHandle;

class TemplateActionServer
{
    public:

    TemplateActionServer() : action_server
    (
        nh, "/example_action",
        boost::bind(&TemplateActionServer::goal_callback, this, _1),
        boost::bind(&TemplateActionServer::cancel_callback, this, _1),
        false
    )
    {
        action_server.start();

        ROS_INFO("Action server is up");
    }

    private:

    void execute_callback(GoalHandle goal_handle)
    {
        ROS_INFO("Executing new goal");

        int goal = goal_handle.getGoal()->goal;
        custom_msgs::ExampleResult result;
        custom_msgs::ExampleFeedback feedback;
        bool success = false;
        bool cancel = false;

        std::string goal_id = goal_handle.getGoalID().id;

        ros::Rate rate(1.0);

        int counter = 0;
        int max_num = 10;

        while (ros::ok())
        {
            if (cancel_goals[goal_id])
            {
                cancel = true;
                break;
            }

            if (counter == goal)
            {
                success = true;
                break;
            }

            if (counter >= max_num)
            {
                break;
            }

            counter++;

            feedback.feedback = counter;
            goal_handle.publishFeedback(feedback);

            rate.sleep();
        }

        result.result = counter;

        if (cancel)
        {
            goal_handle.setCanceled(result);
            ROS_WARN("Goal canceled");
        }
        else if (success)
        {
            goal_handle.setSucceeded(result);
            ROS_INFO("Goal succeeded");
        }
        else
        {
            goal_handle.setAborted(result);
            ROS_ERROR("Goal aborted");
        }

        cancel_goals.erase(goal_id);
    }

    void goal_callback(GoalHandle goal_handle)
    {
        // Uncomment to enable a server with a single goal at the same time
        /* if (cancel_goals.size() > 0)
        {
            goal_handle.setRejected();
            ROS_WARN("A goal is currently active. Incoming goal rejected");
            return;
        } */
        
        if (goal_handle.getGoal()->goal > 0)
        {
            goal_handle.setAccepted();
            ROS_INFO("Incoming goal accepted");
        }
        else
        {
            goal_handle.setRejected();
            ROS_INFO("Incoming goal rejected");
            return;
        }

        cancel_goals[goal_handle.getGoalID().id] = false;

        std::thread(&TemplateActionServer::execute_callback, this, goal_handle).detach();
    }

    void cancel_callback(GoalHandle goal_handle)
    {
        std::string goal_id = goal_handle.getGoalID().id;

        if (cancel_goals.find(goal_id) != cancel_goals.end())
        {
            cancel_goals[goal_id] = true;
            ROS_WARN("Canceling goal");
        }
    }

    ros::NodeHandle nh;
    actionlib::ActionServer<custom_msgs::ExampleAction> action_server;
    std::map<std::string, bool> cancel_goals {};
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "action_server");
    TemplateActionServer action_server;
    ros::spin();

    return 0;

}