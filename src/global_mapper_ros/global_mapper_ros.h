// Copyright 2017 Massachusetts Institute of Technology
#pragma once

#include "ros/ros.h"

namespace global_mapper {

class GlobalMapperRos {
 public:
  GlobalMapperRos();
  ~GlobalMapperRos() = default;

 private:
  void GetParams();
  void InitSubscribers();
  void InitPublishers();
  void Run();

  bool test_param_;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
};
}  // namespace global_mapper
