#include "UdacityTut.h"

UdacityTut::UdacityTut(ros::NodeHandle &nh) : nh_(nh) {
  // Publishers
  //// Point Clouds
  pub_cloud_raw_ = nh.advertise<sensor_msgs::PointCloud2>
    ("/cloud_raw", 1);
  pub_cloud_downsampled_ = nh.advertise<sensor_msgs::PointCloud2>
    ("/cloud_downsampled", 1);
  pub_cloud_groundless_ = nh.advertise<sensor_msgs::PointCloud2>
    ("/cloud_groundless", 1);

  // Subscribers
  sub_velodyne_points_ = nh.subscribe("/velodyne_points", 1,
                                      &UdacityTut::CallbackLaser, this);
}

void
UdacityTut::CallbackLaser(const sensor_msgs::PointCloud2ConstPtr &msg_cloud) {
  // Convert Point Cloud Message to PCL Point Cloud format
  Cloud::Ptr cloud_in(new Cloud);
  pcl::fromROSMsg(*msg_cloud, *cloud_in);
  RosRelated::PublishCloud(cloud_in, pub_cloud_raw_);

  // Downsample it
  Cloud::Ptr cloud_ds = PclStuff::Downsample(cloud_in, 0.2f);
  RosRelated::PublishCloud(cloud_ds, pub_cloud_downsampled_);

  // Remove ground from the downsampled point cloud
  Cloud::Ptr cloud_groundless = PclStuff::GroundRemover(cloud_ds, 0.5f);
  RosRelated::PublishCloud(cloud_groundless, pub_cloud_groundless_);
}