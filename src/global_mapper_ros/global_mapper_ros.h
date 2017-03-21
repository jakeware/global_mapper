// Copyright 2017 Massachusetts Institute of Technology
#pragma once

#include <csignal>
#include <memory>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
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
  void InitMap();
  void PublishMap(const ros::TimerEvent& event);

  // callbacks
  // void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_ptr);
  void PointCloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_ptr);

  // publishers
  ros::Publisher map_pub_;
  ros::Timer map_pub_timer_;
  ros::Publisher marker_pub_;

  // subscribers
  // message_filters::Subscriber<sensor_msgs::PointCloud2> point_cloud_sub_;
  ros::Subscriber point_cloud_sub_;

  // params
  double voxel_xyz0_[3];
  double voxel_xyz1_[3];
  double voxel_meters_per_pixel_[3];
  double voxel_init_value_;

  GlobalMapper global_mapper_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::Buffer tf_buffer_;
  // std::shared_ptr<tf::MessageFilter<sensor_msgs::PointCloud2> > tf_filter_;
};
}  // namespace global_mapper
