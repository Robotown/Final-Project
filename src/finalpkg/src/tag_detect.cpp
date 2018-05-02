#include <ros/ros.h>
#include <logical_camera_plugin/logicalImage.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <vector>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/impl/utils.h>
#include <tf2_ros/transform_listener.h>

std::vector<std::string> found_tags;

tf2_ros::Buffer buffer;
namespace tf2
{
    inline
    void convert(const geometry_msgs::TransformStamped& transform, geometry_msgs::PoseStamped& pose)
    {
        pose.pose.orientation = transform.transform.rotation;
        pose.pose.position.x = transform.transform.translation.x;
        pose.pose.position.y = transform.transform.translation.y;
        pose.pose.position.z = transform.transform.translation.z;
        pose.header = transform.header;
    }
}

void camera_callback(const logical_camera_plugin::logicalImage &image)
{
    if (std::find(found_tags.begin(), found_tags.end(), image.modelName) == found_tags.end()) {
        found_tags.push_back(image.modelName);
        
        tf2::Vector3 image_position = tf2::Vector3(image.pose_pos_x,
                                                   image.pose_pos_y,
                                                   image.pose_pos_z);
        
        tf2::Quaternion image_orientation = tf2::Quaternion(image.pose_rot_x,
                                                            image.pose_rot_y,
                                                            image.pose_rot_z,
                                                            image.pose_rot_w);

        geometry_msgs::TransformStamped image_transform;
        tf2::convert( tf2::Transform(image_orientation, image_position, ros::Duration(1.0)), image_transform.transform );

        image_transform.header.stamp = ros::Time::now();
        image_transform.header.frame_id = "base_link";
        image_transform.child_frame_id = image.modelName;
        
        geometry_msgs::PoseStamped tag_pose;
        tf2::convert( buffer.transform(image_transform, "map"), tag_pose );
        
        
        ROS_INFO_STREAM("Found tag: " << image.modelName << "\n" << tag_pose << "\n" <<
                        "Total is now: " << found_tags.size() );
    }

}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "tag_detect");
    ros::NodeHandle nh;

    ros::Subscriber subCamera = nh.subscribe("/objectsDetected", 1000, &camera_callback);

    tf2_ros::TransformListener tf_listener(buffer, nh);

    ros::spin();

    return 0;
}
