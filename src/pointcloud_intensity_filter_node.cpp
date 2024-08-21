#include "pointcloud_intensity_filter/pointcloud_intensity_filter_node.h"

PointCloudIntensityFilter::PointCloudIntensityFilter(ros::NodeHandle &nh) : nh_(nh)
{
    nh_.param("pointcloud_intensity_filter_node/intensity_threshold", intensity_threshold_, 0.0f);
    nh_.param("pointcloud_intensity_filter_node/input_topic", input_topic, std::string("/ouster/points"));
    nh_.param("pointcloud_intensity_filter_node/output_topic", output_topic, std::string("/ouster/filtered_points"));

    sub_ = nh_.subscribe("/ouster/points", 1, &PointCloudIntensityFilter::pointCloudCallback, this);
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/ouster/filtered_points", 1);
}

void PointCloudIntensityFilter::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    // Initialize the output PointCloud2 message
    sensor_msgs::PointCloud2 output;
    output.header = cloud_msg->header;
    output.height = cloud_msg->height;  // Keep the original height
    output.width = 0;  // We will update this later based on the number of filtered points
    output.fields = cloud_msg->fields;
    output.is_bigendian = cloud_msg->is_bigendian;
    output.point_step = cloud_msg->point_step;
    output.row_step = cloud_msg->row_step;  // Will be recalculated later
    output.is_dense = cloud_msg->is_dense;
    
    // Reserve space for filtered points
    output.data.reserve(cloud_msg->data.size());

    // Iterate through the point cloud data
    size_t filtered_points_count = 0;
    for (size_t i = 0; i < cloud_msg->width * cloud_msg->height; ++i)
    {
        float intensity;
        memcpy(&intensity, &cloud_msg->data[i * cloud_msg->point_step + 16], sizeof(float));  // 16 is the offset for intensity

        if (intensity > intensity_threshold_)
        {
            // Copy the entire point data (all fields) to the output data
            const uint8_t* start = &cloud_msg->data[i * cloud_msg->point_step];
            const uint8_t* end = start + cloud_msg->point_step;
            output.data.insert(output.data.end(), start, end);

            // Count the filtered points
            filtered_points_count++;
        }
    }

    // Update the width and row_step
    output.width = filtered_points_count / output.height;
    output.row_step = output.width * output.point_step;

    // Adjust the actual size of the data buffer
    output.data.resize(filtered_points_count * output.point_step);

    // Publish the filtered point cloud
    pub_.publish(output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_intensity_filter_node");
    ros::NodeHandle nh;

    PointCloudIntensityFilter filter(nh);

    ros::spin();
    return 0;
}
