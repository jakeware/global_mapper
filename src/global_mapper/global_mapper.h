// Copyright 2017 Massachusetts Institute of Technology
#pragma once

#include <deque>
#include <vector>

#include "pcl_ros/point_cloud.h"

namespace global_mapper {

class GlobalMapper {
 public:
  GlobalMapper();
  ~GlobalMapper();
  void PushPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& point_cloud);

 private:
  std::deque<pcl::PointCloud<pcl::PointXYZ>::ConstPtr > point_cloud_buffer_;
  int global_map_size_x_;
  int global_map_size_y_;
  int global_map_size_z_;
  std::vector<float> global_map_;
};
}  // namespace global_mapper
