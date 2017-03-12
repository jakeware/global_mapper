// Copyright 2017 Massachusetts Institute of Technology
#pragma once

#include "ros/ros.h"

#include "pcl_ros/point_cloud.h"

namespace global_mapper {

class GlobalMapperRos {
 public:
  GlobalMapperRos();
  ~GlobalMapperRos() = default;

 private:
  void GetParams();
  void InitSubscribers();
  void InitPublishers();
  void Run();

  // callbacks
  void PointCloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg);

  // publishers

  // subscribers
  ros::Subscriber pointcloud_sub_;

  // params
  bool test_param_;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
};
}  // namespace global_mapper
