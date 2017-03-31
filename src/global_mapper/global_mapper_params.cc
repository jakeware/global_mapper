// Copyright 2017 Massachusetts Institute of Technology

#include "global_mapper/global_mapper_params.h"

namespace global_mapper {

Params::Params()
  : voxel_xyz_min_(3, 0.0),
    voxel_xyz_max_(3, 0.0),
    voxel_resolution_(3, 0.0),
    voxel_init_value_(0.0),
    voxel_min_range_(0.0),
    voxel_max_range_(0.0),
    voxel_min_z_abs_(0.0),
    voxel_max_z_abs_(0.0),
    voxel_use_rel_cropping_(false),
    voxel_min_z_rel_(0.0),
    voxel_max_z_rel_(0.0) {
  // nothing
}
}  // namespace global_mapper
