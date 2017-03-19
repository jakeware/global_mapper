// Copyright 2017 Massachusetts Institute of Technology
#pragma once

#include <csignal>
#include <deque>
#include <vector>
#include <thread>
#include <mutex>

#include "pcl_ros/point_cloud.h"

namespace global_mapper {

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class GlobalMapper {
 public:
  explicit GlobalMapper(volatile std::sig_atomic_t* stop_signal_ptr_);
  ~GlobalMapper();

  // copy constructors
  GlobalMapper(const GlobalMapper& rhs) = delete;
  GlobalMapper& operator=(const GlobalMapper& rhs) = delete;

  // move constructors
  GlobalMapper(GlobalMapper&& rhs) = delete;
  GlobalMapper& operator=(GlobalMapper&& rhs) = delete;

  const void IndToCoord(int ind, int ixyz[3]);
  void PushPointCloud(const PointCloud::ConstPtr& cloud_ptr);
  void Run();

  std::mutex cloud_mutex_;
  std::mutex map_mutex_;
  volatile std::sig_atomic_t* stop_signal_ptr_;
  std::vector<float> global_map_;
  int ixyz_max_[3];

 private:
  const int CoordToInd(int ixyz[3]);
  const PointCloud::ConstPtr PopPointCloud();
  const PointCloud::ConstPtr TransformPointCloud(const PointCloud::ConstPtr& point_cloud);
  void InsertPointCloud(const PointCloud::ConstPtr& point_cloud);
  void Spin();

  std::deque<PointCloud::ConstPtr > point_cloud_buffer_;

  std::thread thread_;
};
}  // namespace global_mapper
