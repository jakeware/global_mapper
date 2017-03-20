// Copyright 2017 Massachusetts Institute of Technology

#include "global_mapper/global_mapper.h"

namespace global_mapper {

GlobalMapper::GlobalMapper(volatile std::sig_atomic_t* stop_signal_ptr) :
  stop_signal_ptr_(stop_signal_ptr),
  ixyz_max_(),
  global_map_() {
  ixyz_max_[0] = 100;
  ixyz_max_[1] = 100;
  ixyz_max_[2] = 10;
  global_map_.resize(ixyz_max_[0]*ixyz_max_[1]*ixyz_max_[2], 0.0);
  fprintf(stderr, "global_map.size: %lu\n", global_map_.size());
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

const int GlobalMapper::CoordToInd(int ixyz[3]) {
  return (static_cast<int>(ixyz[0])
          + static_cast<int>(ixyz[1])*ixyz_max_[0]
          + static_cast<int>(ixyz[2])*ixyz_max_[0]*ixyz_max_[1]);
}

const void GlobalMapper::IndToCoord(int ind, int ixyz[3]) {
  // z
  ixyz[2] = ind / (ixyz_max_[0] * ixyz_max_[1]);
  ind -= ixyz[2] * (ixyz_max_[0] * ixyz_max_[1]);

  // y
  ixyz[1] = ind / (ixyz_max_[0]);
  ind -= ixyz[1] * ixyz_max_[0];

  // x
  ixyz[0] = ind;
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
  int ixyz[3] = {0};
  for (int i = 0; i < cloud_ptr->points.size(); i++) {
    ixyz[0] = cloud_ptr->points[i].x;
    ixyz[1] = cloud_ptr->points[i].y;
    ixyz[2] = cloud_ptr->points[i].z;

    // check bounds and insert
    if (ixyz[0] < 0 || ixyz[0] >= ixyz_max_[0] || ixyz[1] < 0 || ixyz[1] >= ixyz_max_[1]
        || ixyz[2] < 0 || ixyz[2] >= ixyz_max_[2]) {
      continue;
    } else {
      global_map_[CoordToInd(ixyz)] = 1.0;
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
  }
}

void GlobalMapper::Run() {
  fprintf(stderr, "GlobalMapper::Run");
  thread_ = std::thread(&GlobalMapper::Spin, this);
}

}  // namespace global_mapper
