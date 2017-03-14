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

void GlobalMapper::PushPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& point_cloud) {
  ROS_INFO("PushPointCloud");
  point_cloud_buffer_.push_front(point_cloud);
  ROS_INFO("point_cloud_buffer.size(): %lu", point_cloud_buffer_.size());
}

void GlobalMapper::Run() {
  fprintf(stderr, "GlobalMapper::Run");

  while (!(*stop_signal_ptr_)) {
    // nothing
  }
}

}  // namespace global_mapper
