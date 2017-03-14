// Copyright 2017 Massachusetts Institute of Technology

#include <csignal>

#include "ros/ros.h"

#include "pcl_ros/point_cloud.h"
// #include "pcl_conversions/pcl_conversions.h"
// #include "pcl/point_types.h"

#include "fla_utils/param_utils.h"

#include "global_mapper_ros/global_mapper_ros.h"
#include "global_mapper/global_mapper.h"

namespace global_mapper {

GlobalMapperRos::GlobalMapperRos(volatile std::sig_atomic_t* stop_signal_ptr) :
  stop_signal_ptr_(stop_signal_ptr),
  test_param_(false),
  global_mapper_(stop_signal_ptr),
  nh_(),
  pnh_("~") {
}

GlobalMapperRos::~GlobalMapperRos() {
  if (thread_.joinable()) {
    thread_.join();
  }
}

void GlobalMapperRos::GetParams() {
  fla_utils::SafeGetParam(pnh_, "test_param", test_param_);
}

void GlobalMapperRos::InitSubscribers() {
  ROS_INFO("InitSubscribers");
  point_cloud_sub_ = nh_.subscribe<pcl::PointCloud<pcl::PointXYZ> >("pointcloud", 10, &GlobalMapperRos::PointCloudCallback, this);
}

void GlobalMapperRos::InitPublishers() {
  // not yet implemented
}

void GlobalMapperRos::PointCloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& point_cloud) {
  ROS_INFO("PointCloudCallback");
  global_mapper_.PushPointCloud(point_cloud);
}

void GlobalMapperRos::Run() {
  GetParams();
  InitSubscribers();
  InitPublishers();

  // start mapping thread
  thread_ = std::thread(&GlobalMapper::Run, global_mapper_);

  // handle ros callbacks
  ros::spin();
}

}  // namespace global_mapper
