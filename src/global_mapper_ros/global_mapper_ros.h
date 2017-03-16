// Copyright 2017 Massachusetts Institute of Technology
#pragma once

#include <csignal>
#include <memory>

#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "sensor_msgs/PointCloud2.h"
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
  void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_ptr);

  // publishers

  // subscribers
  message_filters::Subscriber<sensor_msgs::PointCloud2> point_cloud_sub_;
  // ros::Subscriber point_cloud_sub_;

  // params
  bool test_param_;

  GlobalMapper global_mapper_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf::TransformListener tf_listener_;
  std::shared_ptr<tf::MessageFilter<sensor_msgs::PointCloud2> > tf_filter_;
};
}  // namespace global_mapper
