// Copyright 2017 Massachusetts Institute of Technology
#pragma once

#include <vector>
#include <string>

namespace global_mapper {
struct Params {
  std::string map_frame;

  // pixel map
  double pixel_xy_min[2];
  double pixel_xy_max[2];
  double pixel_resolution;
  double pixel_init_value;
  double pixel_bound_min;
  double pixel_bound_max;
  double pixel_min_z_abs;
  double pixel_max_z_abs;
  bool pixel_use_rel_flatten;
  double pixel_min_z_rel;
  double pixel_max_z_rel;

  // voxel map
  double voxel_origin[3];
  double voxel_world_dimensions[3];
  std::vector<double> voxel_resolution;
  double voxel_init_value;
  double voxel_bound_min;
  double voxel_bound_max;
  double voxel_min_z_abs;
  double voxel_max_z_abs;
  bool voxel_use_rel_cropping;
  double voxel_min_z_rel;
  double voxel_max_z_rel;
  double voxel_hit_inc;
  double voxel_miss_inc;
};
}  // namespace global_mapper
