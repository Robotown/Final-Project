#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <frontier_exploration/ExploreTaskAction.h>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "explore");

    actionlib::SimpleActionClient<frontier_exploration::ExploreTaskAction> ac("explore_server", true);

    ROS_INFO_STREAM("Waiting for action server...");
    ac.waitForServer();
    ROS_INFO_STREAM("Connected! Sending goal...");

    frontier_exploration::ExploreTaskGoal goal;

    geometry_msgs::PolygonStamped boundary;

    geometry_msgs::Point32 p1, p2, p3, p4, p5;

    p1.x =  10.0; p1.y =  10.0; p1.z = 0.0;
    p2.x =  10.0; p2.y = -10.0; p2.z = 0.0;
    p3.x = -10.0; p3.y = -10.0; p3.z = 0.0;
    p4.x = -10.0; p4.y =  10.0; p4.z = 0.0;
    p5.x =  10.0; p5.y =  10.0; p5.z = 0.0;

    boundary.header.seq = 0;
    boundary.header.stamp = ros::Time::now();
    boundary.header.frame_id = "map";

    boundary.polygon.points.push_back(p1);
    boundary.polygon.points.push_back(p2);
    boundary.polygon.points.push_back(p3);
    boundary.polygon.points.push_back(p4);
    boundary.polygon.points.push_back(p5);

    goal.explore_center.header.frame_id = "map";
    goal.explore_center.point.x = 0.0;
    goal.explore_center.point.y = 0.0;
    goal.explore_center.point.z = 0.0;
    goal.explore_boundary = boundary;
    ac.sendGoal(goal);

    return 0;

}
