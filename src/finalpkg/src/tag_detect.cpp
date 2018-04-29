#include <ros/ros.h>
#include <logical_camera_plugin/logicalImage.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <vector>
#include <iostream>
#include <algorithm>
#include <cmath>

geometry_msgs::Pose robot_pose;
std::vector<std::string> found_tags;

double correct_angle(double angle)
{
    while (angle > 2*M_PI)
        angle -= 2*M_PI;
    while (angle < 0)
        angle += 2*M_PI;
    return angle;
}

void camera_callback(const logical_camera_plugin::logicalImage &image)
{
    if (std::find(found_tags.begin(), found_tags.end(), image.modelName) == found_tags.end()) {
        found_tags.push_back(image.modelName);

        geometry_msgs::Pose tag_pose;
        tag_pose.position.x = image.pose_pos_x + robot_pose.position.x;
        tag_pose.position.y = image.pose_pos_y + robot_pose.position.y;
        tag_pose.position.z = image.pose_pos_z + robot_pose.position.z;

        tag_pose.orientation.x = correct_angle(image.pose_rot_x + robot_pose.orientation.x);
        tag_pose.orientation.y = correct_angle(image.pose_rot_y + robot_pose.orientation.y);
        tag_pose.orientation.z = correct_angle(image.pose_rot_z + robot_pose.orientation.z);
        tag_pose.orientation.w = correct_angle(image.pose_rot_w + robot_pose.orientation.w);

        ROS_INFO_STREAM("Found tag: " << image.modelName << '\n' << tag_pose);

        ROS_INFO_STREAM("\n-----DEBUG-----" <<
                        "ROBOT_POSE:\n" << robot_pose << "\n\n" <<
                        "TAG_POSE:\n" << tag_pose << "\n");
    }
}

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped &pose)
{
    robot_pose = pose.pose.pose;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "tag_detect");
    ros::NodeHandle nh;

    ros::Subscriber subPose = nh.subscribe("/amcl_pose", 1000, &pose_callback);
    ros::Subscriber subCamera = nh.subscribe("/objectsDetected", 1000, &camera_callback);

    ros::spin();

    return 0;
}
