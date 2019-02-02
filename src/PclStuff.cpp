#include "PclStuff.h"

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<pcl::PointXYZI>;


Cloud::Ptr PclStuff::Downsample(const Cloud::ConstPtr &cloud_in,
                                float leaf_size) {
  Cloud::Ptr cloud_result(new Cloud());

  pcl::VoxelGrid<Point> grid;
  grid.setLeafSize(leaf_size, leaf_size, leaf_size);
  grid.setInputCloud(cloud_in);
  grid.filter(*cloud_result);

  return cloud_result;
}

Cloud::Ptr
PclStuff::GroundRemover(Cloud::ConstPtr cloud_in, float treshold) {
  Cloud::Ptr cloud_groundless(new Cloud);
  pcl::SACSegmentation<Point> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(treshold);
  seg.setInputCloud(cloud_in);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.empty()) {
    std::cout
      << "Could not estimate a planar model for the given dataset."
      << std::endl;
    *cloud_groundless = *cloud_in;
    return cloud_groundless;
  }

  pcl::ExtractIndices<Point> extract;
  extract.setInputCloud(cloud_in);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloud_groundless);
  return cloud_groundless;
}