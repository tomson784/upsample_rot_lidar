// Headers in this package
#include <upsample_rot_lidar/upsample_rot_lidar.hpp>

// Headers in ROS
#include <ros/ros.h>

// Headers in Glog
#include <glog/logging.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "upsample_rot_lidar_node");
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    UpsampleRotLidar converter(nh,pnh);
    ros::spin();
    return 0;
}