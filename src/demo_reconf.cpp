#include <iostream>
// For disable PCL complile lib, to use PointXYZIR
#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include <signal.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <patchworkpp/PatchworkppConfig.h>
#include "patchworkpp/patchworkpp.hpp"

using PointType = pcl::PointXYZI;

std::unique_ptr<PatchWorkpp<PointType>> PatchworkppGroundSeg;

ros::Publisher pub_cloud;
ros::Publisher pub_ground;
ros::Publisher pub_non_ground;

template<typename T>
sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<T> cloud, const ros::Time& stamp, std::string frame_id = "map") {
    sensor_msgs::PointCloud2 cloud_ROS;
    pcl::toROSMsg(cloud, cloud_ROS);
    cloud_ROS.header.stamp = stamp;
    cloud_ROS.header.frame_id = frame_id;
    return cloud_ROS;
}

// Callback function for parameter changes
void reconfigureCallback(const patchworkpp::PatchworkppConfig& config, uint32_t level)
{
  // Update your parameters based on the changes
  // You can access the parameters using config.param_name
  // For example: int parameter = config.int_param;
  // Your code to handle parameter changes goes here
  ROS_INFO("Updated parameters");
}

void callbackCloud(const sensor_msgs::PointCloud2::Ptr &cloud_msg)
{
    double time_taken;

    pcl::PointCloud<PointType> pc_curr;
    pcl::PointCloud<PointType> pc_ground;
    pcl::PointCloud<PointType> pc_non_ground;

    pcl::fromROSMsg(*cloud_msg, pc_curr);

    if (PatchworkppGroundSeg->estimate_ground(pc_curr, pc_ground, pc_non_ground, time_taken)) {
        ROS_INFO_STREAM("\033[1;32m" << "Input PointCloud: " << pc_curr.size() << " -> Ground: " << pc_ground.size() <<  "/ NonGround: " << pc_non_ground.size()
            << " (running_time: " << time_taken << " sec)" << "\033[0m");

        pub_cloud.publish(cloud2msg(pc_curr, cloud_msg->header.stamp, cloud_msg->header.frame_id));
        pub_ground.publish(cloud2msg(pc_ground, cloud_msg->header.stamp, cloud_msg->header.frame_id));
        pub_non_ground.publish(cloud2msg(pc_non_ground, cloud_msg->header.stamp, cloud_msg->header.frame_id));
    }
}

int main(int argc, char**argv) {

    ros::init(argc, argv, "Demo");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string cloud_topic;
    pnh.param<string>("cloud_topic", cloud_topic, "/pointcloud");

    ROS_INFO("Operating patchwork++...");
    PatchworkppGroundSeg.reset(new PatchWorkpp<PointType>());

    pub_cloud       = pnh.advertise<sensor_msgs::PointCloud2>("cloud", 100, true);
    pub_ground      = pnh.advertise<sensor_msgs::PointCloud2>("ground", 100, true);
    pub_non_ground  = pnh.advertise<sensor_msgs::PointCloud2>("nonground", 100, true);

    ros::Subscriber sub_cloud = nh.subscribe(cloud_topic, 100, callbackCloud);

    // Create a dynamic reconfigure server
    dynamic_reconfigure::Server<patchworkpp::PatchworkppConfig> server;
    dynamic_reconfigure::Server<patchworkpp::PatchworkppConfig>::CallbackType f;

    // Set the callback function
    f = boost::bind(&reconfigureCallback, _1, _2);
    server.setCallback(f);

    ros::spin();

    return 0;
}