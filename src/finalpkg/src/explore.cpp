#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <frontier_exploration/ExploreTaskAction.h>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "explore");

    actionlib::SimpleActionClient<frontier_exploration::ExploreTaskAction> ac("object_search_server", true);

    ROS_INFO_STREAM("Waiting for action server...");
    ac.waitForServer();
    ROS_INFO_STREAM("Connected! Sending goal...");

    frontier_exploration::ExploreTaskGoal goal;

    geometry_msgs::PolygonStamped bounds;

    goal.explore_center.header.frame_id = "map";
    goal.explore_center.point.x = 0.0;
    goal.explore_center.point.y = 0.0;
    goal.explore_center.point.z = 0.0;
    goal.explore_boundary = bounds;
    ac.sendGoal(goal);

    return 0;

}
