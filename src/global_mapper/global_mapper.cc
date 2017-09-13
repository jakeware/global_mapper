// Copyright 2017 Massachusetts Institute of Technology

#include "occ_map/voxel_map.hpp"

#include "global_mapper/global_mapper.h"

namespace global_mapper {

GlobalMapper::GlobalMapper(volatile std::sig_atomic_t* stop_signal_ptr, Params& params)
  : stop_signal_ptr_(stop_signal_ptr),
    params_(params),
    data_ready_(0) {
}

GlobalMapper::~GlobalMapper() {
  if (thread_.joinable()) {
    thread_.join();
  }
}

void GlobalMapper::PushPointCloud(const PointCloud::ConstPtr& cloud_ptr) {

  // push
  std::lock_guard<std::mutex> cloud_lock(cloud_mutex());
  point_cloud_buffer_.push_back(cloud_ptr);

  // notify
  std::unique_lock<std::mutex> data_lock(data_mutex());
  data_ready_ = true;
  data_lock.unlock();
  condition_.notify_one();
}

const PointCloud::ConstPtr GlobalMapper::PopPointCloud() {
  // pop
  std::lock_guard<std::mutex> cloud_lock(cloud_mutex());
  PointCloud::ConstPtr cloud_ptr = nullptr;
  if (point_cloud_buffer_.size() > 0) {
    cloud_ptr = point_cloud_buffer_.front();
    point_cloud_buffer_.pop_front();
  }

  // notify
  std::unique_lock<std::mutex> data_lock(data_mutex());
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
  std::lock_guard<std::mutex> map_lock(map_mutex());

  // insert point
  double start[3] = {cloud_ptr->sensor_origin_[0], cloud_ptr->sensor_origin_[1], cloud_ptr->sensor_origin_[2]};
  double end[3] = {0.0};
  float clamp_bounds[2] = {static_cast<float>(params_.voxel_bound_min), static_cast<float>(params_.voxel_bound_max)};
  for (int i = 0; i < cloud_ptr->points.size(); i++) {
    // absolute altitude check
    if ((cloud_ptr->points[i].z > params_.voxel_max_z_abs) ||
        (cloud_ptr->points[i].z < params_.voxel_min_z_abs)) {
      continue;
    }

    // relative altitude check
    if (params_.voxel_use_rel_cropping &&
        ((cloud_ptr->points[i].z > (start[2] + params_.voxel_max_z_rel)) ||
         (cloud_ptr->points[i].z < (start[2] + params_.voxel_min_z_rel)))) {
      continue;
    }

    // insert
    end[0] = cloud_ptr->points[i].x;
    end[1] = cloud_ptr->points[i].y;
    end[2] = cloud_ptr->points[i].z;
    voxel_map_ptr_->raytrace(start, end, -0.1, 0.1, clamp_bounds);
  }
}

void GlobalMapper::FlattenMap() {
  int ixyz[3] = {0};
  double xyz[3] = {0.0};
  double xy[2] = {0.0};
  float occ = 0.0;
  float occ_temp = 0.0;
  for (int i = 0; i < voxel_map_ptr_->dimensions[0]; ++i) {
    ixyz[0] = i;

    for (int j = 0; j < voxel_map_ptr_->dimensions[1]; ++j) {
      ixyz[1] = j;

      // reset mean
      occ = 0.0;
      occ_temp = 0.0;

      for (int k = 0; k < voxel_map_ptr_->dimensions[2]; ++k) {
        ixyz[2] = k;

        // get coordinates from voxel_map
        voxel_map_ptr_->tableToWorld(ixyz, xyz);

        // check bounds
        if (xyz[2] < params_.pixel_min_z_abs ||
            xyz[2] > params_.pixel_max_z_abs) {
          continue;
        }

        // get max over z
        occ_temp = voxel_map_ptr_->readValue(ixyz);
        if (occ_temp > occ) {
          occ = occ_temp;
        }
      }

      // insert into pixel map by position
      xy[0] = xyz[0];
      xy[1] = xyz[1];
      pixel_map_ptr_->writeValue(xy, occ);
    }
  }
}

void GlobalMapper::Spin() {
  PointCloud::ConstPtr cloud_ptr;
  while (!(*stop_signal_ptr_)) {
    std::unique_lock<std::mutex> data_lock(data_mutex());
    condition_.wait(data_lock, [this]{return data_ready_;});
    data_lock.unlock();

    cloud_ptr = PopPointCloud();
    // cloud_ptr = TransformPointCloud(cloud_ptr);
    InsertPointCloud(cloud_ptr);
    FlattenMap();
  }
}

void GlobalMapper::Run() {
  fprintf(stderr, "GlobalMapper::Run");

  printf("min: %0.2f, %0.2f, %0.2f\n", params_.voxel_xyz_min[0], params_.voxel_xyz_min[1], params_.voxel_xyz_min[2]);
  voxel_map_ptr_ = std::make_shared<occ_map::VoxelMap<float> >(params_.voxel_xyz_min,
                                                               params_.voxel_xyz_max,
                                                               params_.voxel_resolution.data(),
                                                               params_.voxel_init_value);

  pixel_map_ptr_ = std::make_shared<occ_map::PixelMap<float> >(params_.pixel_xy_min,
                                                                params_.pixel_xy_max,
                                                                params_.pixel_resolution,
                                                                params_.pixel_init_value);

  thread_ = std::thread(&GlobalMapper::Spin, this);
}

}  // namespace global_mapper
