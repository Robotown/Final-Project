#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <frontier_exploration/ExploreTaskAction.h>
#include <geometry_msgs/Twist.h>
#include <vector>

void addRegions(std::vector<geometry_msgs::PolygonStamped> &regions)
{

    geometry_msgs::Point32 p1, p2, p3, p4, p5;

    geometry_msgs::PolygonStamped region;

    p1.x = 10.0; p1.y = 10.0; p1.z = 0.0;
    p2.x = 10.0; p2.y = 3.34; p2.z = 0.0;
    p3.x = 3.34; p3.y = 3.34; p3.z = 0.0;
    p4.x = 3.34; p4.y = 10.0; p4.z = 0.0;
    p5.x = 10.0; p5.y = 10.0; p5.z = 0.0;

    region.polygon.points.push_back(p1);
    region.polygon.points.push_back(p2);
    region.polygon.points.push_back(p3);
    region.polygon.points.push_back(p4);
    region.polygon.points.push_back(p5);

    regions.push_back(region);

    for(int i = 1; i < 9; i++){
        if(region.polygon.points[0].y > -3.0){

            region.polygon.points[0].y -= 6.66;
            region.polygon.points[1].y -= 6.66;
            region.polygon.points[2].y -= 6.66;
            region.polygon.points[3].y -= 6.66;

        }else{

            region.polygon.points[0].y = 10.0;
            region.polygon.points[3].y = 10.0;

            region.polygon.points[1].y = 3.34;
            region.polygon.points[2].y = 3.34;

            region.polygon.points[0].x -= 6.66;
            region.polygon.points[1].x -= 6.66;
            region.polygon.points[2].x -= 6.66;
            region.polygon.points[3].x -= 6.66;

        }
        region.polygon.points[4] = region.polygon.points[0];
        regions.push_back(region);
    }

    std::swap(regions[0], regions[4]);
    std::swap(regions[3], regions[5]);
    std::swap(regions[4], regions[8]);
    std::swap(regions[5], regions[7]);

}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "explore");
    ros::NodeHandle nh;

    actionlib::SimpleActionClient<frontier_exploration::ExploreTaskAction> ac("explore_server", true);

    ROS_INFO_STREAM("Waiting for action server...");
    ac.waitForServer();
    ROS_INFO_STREAM("Connected! Sending goal...");

    frontier_exploration::ExploreTaskGoal goal;

    std::vector<geometry_msgs::PolygonStamped> regions;

    addRegions(regions);

    ros::Publisher pubTwist = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1000);

    geometry_msgs::Twist twistMsg;

    twistMsg.linear.x = -10.0;

    int i = 0;

    regions[i].header.stamp = ros::Time::now();
    regions[i].header.frame_id = "map";
    goal.explore_center.header.frame_id = "map";
    goal.explore_center.point.x = 0.0;
    goal.explore_center.point.y = 0.0;
    goal.explore_center.point.z = 0.0;
    goal.explore_boundary = regions[i];
    ac.sendGoal(goal);

    ros::Rate rate(2);

    while(i < 9){

        if(ac.getState().toString() == "SUCCEEDED"){

            i++;
            regions[i].header.stamp = ros::Time::now();
            regions[i].header.frame_id = "map";
            goal.explore_center.header.frame_id = "map";

            goal.explore_center.point.x = int (regions[i].polygon.points[0].x + regions[i].polygon.points[2].x) / 2;
            goal.explore_center.point.y = int (regions[i].polygon.points[0].y + regions[i].polygon.points[2].y) / 2;

            goal.explore_boundary = regions[i];

            ac.sendGoal(goal);

        }else if(ac.getState().toString() == "ABORTED"){
            pubTwist.publish(twistMsg); 
            rate.sleep();
            pubTwist.publish(twistMsg); 
            rate.sleep();
            pubTwist.publish(twistMsg); 
            rate.sleep();
            pubTwist.publish(twistMsg); 
            rate.sleep();
            pubTwist.publish(twistMsg); 
            rate.sleep();
            ac.sendGoal(goal);
        }

    }

    return 0;
}
