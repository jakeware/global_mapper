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

  // lock
  std::lock_guard<std::mutex> lock(map_mutex_);

  // insert point
  double start[3] = {cloud_ptr->sensor_origin_[0], cloud_ptr->sensor_origin_[1], cloud_ptr->sensor_origin_[2]};
  double end[3] = {0.0};
  float clamp_bounds[2] = {static_cast<float>(params_.voxel_bound_min_), static_cast<float>(params_.voxel_bound_max_)};
  for (int i = 0; i < cloud_ptr->points.size(); i++) {
    // absolute altitude check
    if ((cloud_ptr->points[i].z > params_.voxel_max_z_abs_) ||
        (cloud_ptr->points[i].z < params_.voxel_min_z_abs_)) {
      continue;
    }

    // relative altitude check
    if (params_.voxel_use_rel_cropping_ &&
        ((cloud_ptr->points[i].z > (start[2] + params_.voxel_max_z_rel_)) ||
         (cloud_ptr->points[i].z < (start[2] + params_.voxel_min_z_rel_)))) {
      continue;
    }

    // insert
    end[0] = cloud_ptr->points[i].x;
    end[1] = cloud_ptr->points[i].y;
    end[2] = cloud_ptr->points[i].z;
    voxel_map_ptr_->raytrace(start, end, -0.1, 0.1, clamp_bounds);
  }
}

void GlobalMapper::FlattenPointCloud() {
  int ixyz[3] = {0};
  double xyz[3] = {0.0};
  double xy[2] = {0.0};
  float occ_mean = 0.0;
  uint8_t occ_mean_byte = 0;
  for (int i=0; i < voxel_map_ptr_->dimensions[0]; ++i) {
    // reset mean
    occ_mean = 0.0;

    for (int j=0; j < voxel_map_ptr_->dimensions[1]; ++j) {
      for (int k=0; k < voxel_map_ptr_->dimensions[2]; ++k) {
        // calculate average over z
        ixyz[0] = i;
        ixyz[1] = j;
        ixyz[2] = k;
        occ_mean += voxel_map_ptr_->readValue(ixyz);
      }

      // convert to uint8_t
      occ_mean /= static_cast<float>(voxel_map_ptr_->dimensions[2]);
      occ_mean = occ_mean*254.0;  // scale to max value of pixel_map
      occ_mean_byte = static_cast<uint8_t>(occ_mean);

      // get coordinates from voxel_map
      voxel_map_ptr_->tableToWorld(ixyz, xyz);

      // insert into pixel map by position
      xy[0] = xyz[0];
      xy[1] = xyz[1];
      pixel_map_ptr_->writeValue(xy, occ_mean);
    }
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
    FlattenPointCloud();
  }
}

void GlobalMapper::Run() {
  fprintf(stderr, "GlobalMapper::Run");

  voxel_map_ptr_ = std::make_shared<occ_map::VoxelMap<float> >(params_.voxel_xyz0_,
                                                                params_.voxel_xyz1_,
                                                                params_.voxel_meters_per_pixel_,
                                                                params_.voxel_init_value_);

  pixel_map_ptr_ = std::make_shared<occ_map::PixelMap<uint8_t> >(params_.pixel_xy0_,
                                                                params_.pixel_xy1_,
                                                                params_.pixel_meters_per_pixel_,
                                                                params_.pixel_init_value_);

  thread_ = std::thread(&GlobalMapper::Spin, this);
}

}  // namespace global_mapper
