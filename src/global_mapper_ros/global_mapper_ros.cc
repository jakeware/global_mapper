// Copyright 2017 Massachusetts Institute of Technology

#include <csignal>

#include "ros/ros.h"

#include "pcl_ros/point_cloud.h"
// #include "pcl_conversions/pcl_conversions.h"
// #include "pcl/point_types.h"

#include "fla_utils/param_utils.h"

#include "global_mapper_ros/global_mapper_ros.h"

namespace global_mapper {

GlobalMapperRos::GlobalMapperRos() :
  test_param_(false),
  nh_(),
  pnh_("~") {
}

void GlobalMapperRos::GetParams() {
  fla_utils::SafeGetParam(pnh_, "test_param", test_param_);
}

void GlobalMapperRos::InitSubscribers() {
  ROS_INFO("InitSubscribers");
  pointcloud_sub_ = nh_.subscribe<pcl::PointCloud<pcl::PointXYZ> >("pointcloud", 10, &GlobalMapperRos::PointCloudCallback, this);
}

void GlobalMapperRos::InitPublishers() {
  // not yet implemented
}

void GlobalMapperRos::PointCloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg) {
  ROS_INFO("PointCloudCallback");
}

void GlobalMapperRos::Run() {
  GetParams();
  InitSubscribers();
  InitPublishers();

  ros::spin();
}

}  // namespace global_mapper
