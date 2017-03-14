// Copyright 2017 Massachusetts Institute of Technology
#pragma once

#include <csignal>
#include <thread>

#include "ros/ros.h"

#include "pcl_ros/point_cloud.h"

#include "global_mapper/global_mapper.h"

namespace global_mapper {

class GlobalMapperRos {
 public:
  explicit GlobalMapperRos(volatile std::sig_atomic_t* stop_signal_ptr);
  ~GlobalMapperRos();

  void Run();

  volatile std::sig_atomic_t* stop_signal_ptr_;

 private:
  void GetParams();
  void InitSubscribers();
  void InitPublishers();

  // callbacks
  void PointCloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg);

  // publishers

  // subscribers
  ros::Subscriber point_cloud_sub_;

  // params
  bool test_param_;

  GlobalMapper global_mapper_;
  std::thread thread_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
};
}  // namespace global_mapper
