#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <custom_msgs/ExampleAction.h>

class TemplateActionServer
{
    public:

    TemplateActionServer() : action_server(nh, "/example_action", boost::bind(&TemplateActionServer::execute_callback, this, _1), false)
    {
        action_server.start();

        ROS_INFO("Action server is up");
    }

    void execute_callback(const custom_msgs::ExampleGoalConstPtr &goal_handle)
    {
        ROS_INFO("Executing new goal");

        int goal = goal_handle->goal;
        custom_msgs::ExampleResult result;
        custom_msgs::ExampleFeedback feedback;
        bool success = false;
        bool cancel = false;

        ros::Rate rate(1.0);

        int counter = 0;
        int max_num = 10;

        while (ros::ok())
        {
            if (action_server.isPreemptRequested())
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
            action_server.publishFeedback(feedback);

            rate.sleep();
        }

        result.result = counter;

        if (cancel)
        {
            action_server.setPreempted(result);
            ROS_WARN("Goal canceled");
        }
        else if (success)
        {
            action_server.setSucceeded(result);
            ROS_INFO("Goal succeeded");
        }
        else
        {
            action_server.setAborted(result);
            ROS_ERROR("Goal aborted");
        }

    }

    private:

    ros::NodeHandle nh;
    actionlib::SimpleActionServer<custom_msgs::ExampleAction> action_server;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "action_server");
    TemplateActionServer action_server;
    ros::spin();
}