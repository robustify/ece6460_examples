#include <ros/ros.h>
#include "PointCloudExample.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_seg_node");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  ece6460_pointcloud_example::PointCloudExample node(n, pn);

  ros::spin();
}
