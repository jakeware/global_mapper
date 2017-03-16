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

  GlobalMapper(const GlobalMapper& rhs) = delete;
  GlobalMapper& operator=(const GlobalMapper& rhs) = delete;

  GlobalMapper(GlobalMapper&& rhs) = delete;
  GlobalMapper& operator=(GlobalMapper&& rhs) = delete;

  void PushPointCloud(const PointCloud::Ptr& point_cloud);
  void Run();

  volatile std::sig_atomic_t* stop_signal_ptr_;

 private:
  const int CoordToInd(int ixyz[3]);
  const void IndToCoord(int ind, int ixyz[3]);
  const PointCloud::ConstPtr PopPointCloud();
  const PointCloud::ConstPtr TransformPointCloud(const PointCloud::ConstPtr& point_cloud);
  void InsertPointCloud(const PointCloud::ConstPtr& point_cloud);
  void Spin();

  std::deque<PointCloud::ConstPtr > point_cloud_buffer_;
  int ixyz_max_[3];
  std::vector<float> global_map_;

  std::thread thread_;
  std::mutex mutex_;
};
}  // namespace global_mapper
