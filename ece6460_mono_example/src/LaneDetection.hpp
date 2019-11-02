#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <image_geometry/pinhole_camera_model.h>
#include <dynamic_reconfigure/server.h>
#include <ece6460_mono_example/LaneDetectionConfig.h>
#include <visualization_msgs/MarkerArray.h>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/PointCloud.h>

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

namespace ece6460_mono_example
{

class LaneDetection
{
  public:
    LaneDetection(ros::NodeHandle n, ros::NodeHandle pn);

  private:
    void reconfig(LaneDetectionConfig& config, uint32_t level);
    void recvImage(const sensor_msgs::ImageConstPtr& msg);
    void recvCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg);

    void segmentImage(const cv::Mat& raw_img, cv::Mat& bin_img);
    void detectWhite(const cv::Mat& sat_img, const cv::Mat& val_img, cv::Mat& white_bin_img);
    void detectYellow(const cv::Mat& hue_img, const cv::Mat& sat_img, cv::Mat& yellow_bin_img);

    geometry_msgs::Point projectPoint(const image_geometry::PinholeCameraModel& model, const cv::Point2d& p);

    tf2_ros::TransformListener listener_;
    tf2_ros::Buffer buffer_;

    ros::Subscriber sub_image_;
    ros::Subscriber sub_cam_info_;
    ros::Publisher pub_markers_;

    dynamic_reconfigure::Server<LaneDetectionConfig> srv_;
    LaneDetectionConfig cfg_;

    sensor_msgs::CameraInfo camera_info_;
    geometry_msgs::TransformStamped camera_transform_; // Coordinate transformation from footprint to camera
    bool looked_up_camera_transform_;
    pcl::search::Search<pcl::PointXYZ>::Ptr kd_tree_;
};

}
