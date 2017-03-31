// Copyright 2017 Massachusetts Institute of Technology

#include "global_mapper/global_mapper_params.h"

namespace global_mapper {

Params::Params()
  : pixel_xy_min_{0.0, 0.0},
    pixel_xy_max_{0.0, 0.0},
    pixel_resolution_(0.0),
    pixel_init_value_(0.0),
    pixel_bound_min_(0.0),
    pixel_bound_max_(0.0),
    pixel_min_z_abs_(0.0),
    pixel_max_z_abs_(0.0),
    pixel_use_rel_flatten_(false),
    pixel_min_z_rel_(0.0),
    pixel_max_z_rel_(0.0),
    voxel_xyz_min_{0.0},
    voxel_xyz_max_{0.0},
    voxel_resolution_{0.0},
    voxel_init_value_(0.0),
    voxel_bound_min_(0.0),
    voxel_bound_max_(0.0),
    voxel_min_z_abs_(0.0),
    voxel_max_z_abs_(0.0),
    voxel_use_rel_cropping_(false),
    voxel_min_z_rel_(0.0),
    voxel_max_z_rel_(0.0),
    voxel_hit_inc_(0.0),
    voxel_miss_inc_(0.0) {
  // nothing
}
}  // namespace global_mapper
