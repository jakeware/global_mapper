// Copyright 2017 Massachusetts Institute of Technology

#include "occ_map/voxel_map.hpp"

#include "global_mapper/global_mapper.h"

namespace global_mapper {

GlobalMapper::GlobalMapper(volatile std::sig_atomic_t* stop_signal_ptr, GlobalMapperParams& params)
  : stop_signal_ptr_(stop_signal_ptr),
    params_(params) {
}

GlobalMapper::~GlobalMapper() {
  if (thread_.joinable()) {
    thread_.join();
  }
}

void GlobalMapper::PushPointCloud(const PointCloud::ConstPtr& cloud_ptr) {

  // push
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  point_cloud_buffer_.push_back(cloud_ptr);

  // notify
  std::unique_lock<std::mutex> unique_lock(data_mutex_);
  data_ready_ = true;
  unique_lock.unlock();
  condition_.notify_one();
}

const PointCloud::ConstPtr GlobalMapper::PopPointCloud() {
  // pop
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  PointCloud::ConstPtr cloud_ptr = nullptr;
  if (point_cloud_buffer_.size() > 0) {
    cloud_ptr = point_cloud_buffer_.front();
    point_cloud_buffer_.pop_front();
  }

  std::unique_lock<std::mutex> unique_lock(data_mutex_);
  if (point_cloud_buffer_.size() == 0) {
    data_ready_ = false;
  }

  return cloud_ptr;
}

const PointCloud::ConstPtr GlobalMapper::TransformPointCloud(const PointCloud::ConstPtr& cloud_ptr) {
  // check for garbage input
  if (!cloud_ptr) {
    return nullptr;
  }

  // not yet implemented
}

void GlobalMapper::InsertPointCloud(const PointCloud::ConstPtr& cloud_ptr) {
  // check for garbage input
  if (!cloud_ptr) {
    return;
  }

  // fprintf(stderr, "insert cloud size: %lu\n", cloud_ptr->points.size());
  // lock
  std::lock_guard<std::mutex> lock(map_mutex_);

  // insert point
  double start[3] = {cloud_ptr->sensor_origin_[0], cloud_ptr->sensor_origin_[1], cloud_ptr->sensor_origin_[2]};
  double end[3] = {0.0};
  float clamp_bounds[2] = {0.0, 10.0};
  for (int i = 0; i < cloud_ptr->points.size(); i++) {
    end[0] = cloud_ptr->points[i].x;
    end[1] = cloud_ptr->points[i].y;
    end[2] = cloud_ptr->points[i].z;

    // insert
    voxel_map_ptr_->raytrace(start, end, -0.1, 0.1, clamp_bounds);
  }
}

void GlobalMapper::Spin() {
  PointCloud::ConstPtr cloud_ptr;
  while (!(*stop_signal_ptr_)) {
    std::unique_lock<std::mutex> lock(data_mutex_);
    condition_.wait(lock, [this]{return data_ready_;});
    lock.unlock();

    cloud_ptr = PopPointCloud();
    // cloud_ptr = TransformPointCloud(cloud_ptr);
    InsertPointCloud(cloud_ptr);
  }
}

void GlobalMapper::Run() {
  fprintf(stderr, "GlobalMapper::Run");

  voxel_map_ptr_ = std::make_shared<occ_map::VoxelMap<float> >(params_.voxel_xyz0_,
                                                                params_.voxel_xyz1_,
                                                                params_.voxel_meters_per_pixel_,
                                                                params_.voxel_init_value_);

  thread_ = std::thread(&GlobalMapper::Spin, this);
}

}  // namespace global_mapper
