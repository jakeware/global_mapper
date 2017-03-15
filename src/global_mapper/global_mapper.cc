// Copyright 2017 Massachusetts Institute of Technology

#include "global_mapper/global_mapper.h"

namespace global_mapper {

GlobalMapper::GlobalMapper(volatile std::sig_atomic_t* stop_signal_ptr) :
  stop_signal_ptr_(stop_signal_ptr),
  global_map_size_x_(100),
  global_map_size_y_(100),
  global_map_size_z_(10),
  global_map_(global_map_size_x_ * global_map_size_y_ * global_map_size_z_, 0.0) {
  // not yet implemented
}

GlobalMapper::~GlobalMapper() {
  // not yet implemented
}

void GlobalMapper::PushPointCloud(const PointCloud::ConstPtr& cloud_ptr) {
  ROS_INFO("PushPointCloud");
  point_cloud_buffer_.push_front(point_cloud);
  ROS_INFO("point_cloud_buffer.size(): %lu", point_cloud_buffer_.size());
}

// PointCloud::ConstPtr& GlobalMapper::PopPointCloud() {
//   ROS_INFO("PopPointCloud");
//   std::lock_guard<std::mutex> lock(mutex_);
//   if (point_cloud_buffer_.size() > 0) {
//     return point_cloud_buffer_.pop_front();
//   } else {
//     return nullptr;
//   }
// }

// const PointCloud::ConstPtr& GlobalMapper::TransformPointCloud(const PointCloud::ConstPtr& cloud_ptr) {
//   // check for garbage input
//   if (!cloud_ptr) {
//     return nullptr;
//   }
// }

// void GlobalMapper::InsertPointCloud(const PointCloud::ConstPtr& cloud_ptr) {
//   // check for garbage input
//   if (!cloud_ptr) {
//     return;
//   }
// }

void GlobalMapper::Run() {
  // fprintf(stderr, "GlobalMapper::Run");

  // PointCloud::ConstPtr cloud_ptr;
  // while (!(*stop_signal_ptr_)) {
    // cloud_ptr = PopPointCloud();
    // cloud_ptr = TransformPointCloud(cloud_ptr);
    // InsertPointCloud(cloud_ptr);
  // }
}

}  // namespace global_mapper
