#include <ros/ros.h>
#include <logical_camera_plugin/logicalImage.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

void camera_callback(const logical_camera_plugin::logicalImage &image)
{
    return;
}

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped &pose)
{
    return;
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
