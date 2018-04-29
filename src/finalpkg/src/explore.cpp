#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <frontier_exploration/ExploreTaskAction.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "explore");
    ros::NodeHandle nh;

    actionlib::SimpleActionClient<frontier_exploration::ExploreTaskAction> ac("explore_server", true);

    ROS_INFO_STREAM("Waiting for action server...");
    ac.waitForServer();
    ROS_INFO_STREAM("Connected! Sending goal...");

    frontier_exploration::ExploreTaskGoal goal;

    geometry_msgs::PolygonStamped boundary;

    boundary.header.frame_id = "map";
    boundary.header.stamp = ros::Time::now();
        
    goal.explore_center.header.frame_id = "map";
    goal.explore_center.point.x = 0.0;
    goal.explore_center.point.y = 0.0;
    goal.explore_center.point.z = 0.0;
    goal.explore_boundary = boundary;
    ac.sendGoal(goal);

    ros::Publisher pubTwist = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1000);

    geometry_msgs::Twist twistMsg;

    twistMsg.linear.x = -5.0;

    while(nh.ok()){
        if(ac.getState().isDone()){
            ROS_INFO_STREAM("I'm stuck! Recovering...");
            pubTwist.publish(twistMsg);
            pubTwist.publish(twistMsg);
            pubTwist.publish(twistMsg);
            ac.sendGoal(goal);
        }
    }

    return 0;
}
