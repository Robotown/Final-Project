#include <ros/ros.h>
#include <logical_camera_plugin/logicalImage.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <vector>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/impl/utils.h>

geometry_msgs::Pose robot_pose;
std::vector<std::string> found_tags;

double fix_angle(double angle)
{
    while (angle > 2*M_PI)
        angle -= 2*M_PI;
    while (angle < 0)
        angle += 2*M_PI;

    return angle;
}

geometry_msgs::Pose combine_pose_with_image(const geometry_msgs::Pose &pose,
                                            const logical_camera_plugin::logicalImage &image)
{
    geometry_msgs::Pose final_pose;

    tf2::Quaternion image_orientation = tf2::Quaternion(image.pose_rot_x,
                                                        image.pose_rot_y,
                                                        image.pose_rot_z,
                                                        image.pose_rot_w);
    tf2::Quaternion pose_orientation = tf2::impl::toQuaternion(pose.orientation);
    
    double yaw = fix_angle(tf2::impl::getYaw(pose_orientation) +
                           tf2::impl::getYaw(image_orientation));

    if (yaw >= 0 && yaw > M_PI/2) {
        final_pose.position.x = pose.position.x - image.pose_pos_x;
        final_pose.position.y = pose.position.y - image.pose_pos_y;
    }
    else if (yaw >= M_PI/2 && yaw < M_PI) {
        final_pose.position.x = pose.position.x + image.pose_pos_x;
        final_pose.position.y = pose.position.y - image.pose_pos_y;
    }
    else if (yaw >= M_PI && yaw < 3*M_PI/2) {
        final_pose.position.x = pose.position.x + image.pose_pos_x;
        final_pose.position.y = pose.position.y + image.pose_pos_y;
    }
    else {
        final_pose.position.x = pose.position.x - image.pose_pos_x;
        final_pose.position.y = pose.position.y + image.pose_pos_y;
    }

    final_pose.position.z = 0.0;


    tf2::Quaternion temp_orientation;
    temp_orientation.setRPY(0.0, 0.0, yaw);
    tf2::convert(temp_orientation, final_pose.orientation);

    return final_pose;
}

void camera_callback(const logical_camera_plugin::logicalImage &image)
{
    if (std::find(found_tags.begin(), found_tags.end(), image.modelName) == found_tags.end()) {
        found_tags.push_back(image.modelName);

        geometry_msgs::Pose tag_pose = combine_pose_with_image(robot_pose, image);

        ROS_INFO_STREAM("Found tag: " << image.modelName << '\n' << tag_pose);

        ROS_INFO_STREAM("\n-----DEBUG-----" << "\n" <<
                        "ROBOT_POSE:\n" << robot_pose << "\n" <<
                        "TAG_POSE:\n" << image);
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
