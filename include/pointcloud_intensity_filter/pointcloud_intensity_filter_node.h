#ifndef POINTCLOUD_INTENSITY_FILTER_NODE_H
#define POINTCLOUD_INTENSITY_FILTER_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.h>

class PointCloudIntensityFilter
{
public:
    PointCloudIntensityFilter(ros::NodeHandle &nh);
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    float intensity_threshold_;
    std::string input_topic;
    std::string output_topic;
};

#endif  // POINTCLOUD_INTENSITY_FILTER_NODE_H
