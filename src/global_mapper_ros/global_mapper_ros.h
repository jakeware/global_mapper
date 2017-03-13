// Copyright 2017 Massachusetts Institute of Technology
#pragma once

#include <csignal>

#include "ros/ros.h"
#include "pcl_ros/point_cloud.h"

#include "pcl/filters/voxel_grid_occlusion_estimation.h"

namespace global_mapper {

extern volatile std::sig_atomic_t stop_signal_;

class GlobalMapperRos {
 public:
  GlobalMapperRos();
  ~GlobalMapperRos() = default;

  void Run();

  static volatile std::sig_atomic_t stop_signal_;

 private:
  void GetParams();
  void InitSubscribers();
  void InitPublishers();

  // callbacks
  void PointCloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg);

  // publishers

  // subscribers
  ros::Subscriber pointcloud_sub_;

  // pcl
  pcl::VoxelGridOcclusionEstimation<pcl::PointXYZ> voxel_grid_;

  // params
  bool test_param_;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
};
}  // namespace global_mapper
