// Copyright 2017 Massachusetts Institute of Technology

#include "ros/ros.h"
#include "fla_utils/param_utils.h"

#include "global_mapper_ros/global_mapper_ros.h"

namespace global_mapper {

GlobalMapperRos::GlobalMapperRos() :
  test_param_(false),
  nh_(),
  pnh_("~") {
  // not yet implemented
}

void GlobalMapperRos::GetParams() {
  // subscribers

  // publishers

  fla_utils::SafeGetParam(pnh_, "test_param", test_param_);
}

void GlobalMapperRos::Run() {
  GetParams();
}

}  // namespace global_mapper
