// Copyright 2017 Massachusetts Institute of Technology
#pragma once

namespace global_mapper {
class GlobalMapperParams {
 public:
  GlobalMapperParams();

  // pixel map
  double pixel_xy0_[2];
  double pixel_xy1_[2];
  double pixel_meters_per_pixel_;
  double pixel_init_value_;
  double pixel_min_range_;
  double pixel_max_range_;
  double pixel_min_z_abs_;
  double pixel_max_z_abs_;
  bool pixel_use_rel_flatten_;
  double pixel_min_z_rel_;
  double pixel_max_z_rel_;

  // voxel map
  double voxel_xyz0_[3];
  double voxel_xyz1_[3];
  double voxel_meters_per_pixel_[3];
  double voxel_init_value_;
  double voxel_min_range_;
  double voxel_max_range_;
  double voxel_min_z_abs_;
  double voxel_max_z_abs_;
  bool voxel_use_rel_cropping_;
  double voxel_min_z_rel_;
  double voxel_max_z_rel_;
  double voxel_hit_inc_;
  double voxel_miss_inc_;
};
}  // namespace global_mapper
