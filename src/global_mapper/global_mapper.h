// Copyright 2017 Massachusetts Institute of Technology
#pragma once

#include <csignal>
#include <deque>
#include <vector>
#include <mutex>

#include "pcl_ros/point_cloud.h"

namespace global_mapper {

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class GlobalMapper {
 public:
  explicit GlobalMapper(volatile std::sig_atomic_t* stop_signal_ptr_);
  ~GlobalMapper();
  void PushPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& point_cloud);
  void Run();

  volatile std::sig_atomic_t* stop_signal_ptr_;
  std::mutex mutex_;

 private:
  std::deque<pcl::PointCloud<pcl::PointXYZ>::ConstPtr > point_cloud_buffer_;
  int global_map_size_x_;
  int global_map_size_y_;
  int global_map_size_z_;
  std::vector<float> global_map_;
};
}  // namespace global_mapper
