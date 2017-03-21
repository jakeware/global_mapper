// Copyright 2017 Massachusetts Institute of Technology
#pragma once

#include <csignal>
#include <deque>
#include <vector>
#include <thread>
#include <mutex>
#include <memory>
#include <condition_variable>

#include "pcl_ros/point_cloud.h"

#include "occ_map/voxel_map.hpp"
#include "occ_map/pixel_map.hpp"

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

  void PushPointCloud(const PointCloud::ConstPtr& cloud_ptr);
  void InitMap(double voxel_xyz0[3],
               double voxel_xyz1[3],
               double voxel_meters_per_pixel[3],
               double voxel_init_value);
  void Run();

  std::mutex cloud_mutex_;
  std::mutex map_mutex_;
  std::mutex data_mutex_;
  volatile std::sig_atomic_t* stop_signal_ptr_;
  std::shared_ptr<occ_map::VoxelMap<float> > voxel_map_ptr_;
  // occ_map::PixelMap<float> pixel_map_;

 private:
  const PointCloud::ConstPtr PopPointCloud();
  const PointCloud::ConstPtr TransformPointCloud(const PointCloud::ConstPtr& point_cloud);
  void InsertPointCloud(const PointCloud::ConstPtr& point_cloud);
  void Spin();

  std::deque<PointCloud::ConstPtr > point_cloud_buffer_;

  std::thread thread_;
  std::condition_variable condition_;
  std::sig_atomic_t data_ready_;
};
}  // namespace global_mapper
