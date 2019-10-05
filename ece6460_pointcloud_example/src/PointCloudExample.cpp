#include "PointCloudExample.hpp"
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>

namespace ece6460_pointcloud_example
{

  PointCloudExample::PointCloudExample(ros::NodeHandle& n, ros::NodeHandle& pn)
  {
    srv_.setCallback(boost::bind(&PointCloudExample::reconfig, this, _1, _2));

    sub_cloud_ = n.subscribe<sensor_msgs::PointCloud2>("cepton/points_raw", 10, &PointCloudExample::recvCloud, this);
    pub_filtered_cloud_ = n.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 10);
  }

  void PointCloudExample::reconfig(PointCloudExampleConfig& config, uint32_t level)
  {
    cfg_ = config;
  }

  void PointCloudExample::recvCloud(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    // Instantiate point clouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // TODO: Copy ROS message data into PCL cloud

    // Instantiate passthrough filter and array of filtered point indices
    pcl::IndicesPtr roi_indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZ> pass;

    // Give passthrough filter the pointer to the cloud we want to filter
    // and the indices vector to store the selected points
    pass.setInputCloud (input_cloud);
    pass.setIndices (roi_indices);

    // Ask passthrough filter to extract points in a given X range
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (cfg_.x_min, cfg_.x_max);
    pass.filter (*roi_indices);

    // TODO: Ask passthrough filter to extract points in a given Y range

    // TODO: Ask passthrough filter to extract points in a given Z range

    // TODO: Copy filtered cloud data into a ROS message
    
    // TODO: Publish output point cloud
   
  }

}