// Copyright 2017 Massachusetts Institute of Technology
#pragma once

#include <vector>

namespace global_mapper {
class GlobalMapperParams {
 public:
  GlobalMapperParams();

  std::vector<double> voxel_xyz_min_;
  std::vector<double> voxel_xyz_max_;
  std::vector<double> voxel_resolution_;
  double voxel_init_value_;
  double voxel_min_range_;
  double voxel_max_range_;
  double voxel_min_z_abs_;
  double voxel_max_z_abs_;
  bool voxel_use_rel_cropping_;
  double voxel_min_z_rel_;
  double voxel_max_z_rel_;
};
}  // namespace global_mapper
