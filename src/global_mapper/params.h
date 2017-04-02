// Copyright 2017 Massachusetts Institute of Technology
#pragma once

#include <vector>
#include <string>

namespace global_mapper {
struct Params {
  Params() :
    map_frame(""),
    pixel_xy_min{0.0, 0.0},
    pixel_xy_max{0.0, 0.0},
    pixel_resolution(0.0),
    pixel_init_value(0.0),
    pixel_bound_min(0.0),
    pixel_bound_max(0.0),
    pixel_min_z_abs(0.0),
    pixel_max_z_abs(0.0),
    pixel_use_rel_flatten(false),
    pixel_min_z_rel(0.0),
    pixel_max_z_rel(0.0),
    voxel_xyz_min{0.0},
    voxel_xyz_max{0.0},
    voxel_resolution{0.0},
    voxel_init_value(0.0),
    voxel_bound_min(0.0),
    voxel_bound_max(0.0),
    voxel_min_z_abs(0.0),
    voxel_max_z_abs(0.0),
    voxel_use_rel_cropping(false),
    voxel_min_z_rel(0.0),
    voxel_max_z_rel(0.0),
    voxel_hit_inc(0.0),
    voxel_miss_inc(0.0) {
      // nothing
    }

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
  double voxel_xyz_min[3];
  double voxel_xyz_max[3];
  double voxel_resolution[3];
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
