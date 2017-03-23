// Copyright 2017 Massachusetts Institute of Technology

#include "global_mapper/global_mapper_params.h"

namespace global_mapper {

GlobalMapperParams::GlobalMapperParams()
  : voxel_xyz0_{0.0, 0.0, 0.0},
    voxel_xyz1_{0.0, 0.0, 0.0},
    voxel_meters_per_pixel_{0.0, 0.0, 0.0},
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
