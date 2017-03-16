// Copyright 2017 Massachusetts Institute of Technology

#include <csignal>
#include <memory>

#include "ros/ros.h"
#include "pcl_ros/transforms.h"
#include "pcl/conversions.h"

#include "fla_utils/param_utils.h"

#include "global_mapper_ros/global_mapper_ros.h"
#include "global_mapper/global_mapper.h"

namespace global_mapper {

GlobalMapperRos::GlobalMapperRos(volatile std::sig_atomic_t* stop_signal_ptr) :
  stop_signal_ptr_(stop_signal_ptr),
  test_param_(false),
  global_mapper_(stop_signal_ptr),
  nh_(),
  pnh_("~"),
  tf_listener_() {
}

GlobalMapperRos::~GlobalMapperRos() {
  // nothing
}

void GlobalMapperRos::GetParams() {
  fla_utils::SafeGetParam(pnh_, "test_param", test_param_);
}

void GlobalMapperRos::InitSubscribers() {
  ROS_INFO("InitSubscribers");
  point_cloud_sub_.subscribe(nh_, "pointcloud", 10);
  tf_filter_ = std::make_shared<tf::MessageFilter<sensor_msgs::PointCloud2> >(point_cloud_sub_, tf_listener_, "world", 10);
  tf_filter_->registerCallback(boost::bind(&GlobalMapperRos::PointCloudCallback, this, _1));
}

void GlobalMapperRos::InitPublishers() {
  // not yet implemented
}

void GlobalMapperRos::PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_ptr) {
  ROS_INFO("PointCloudCallback");

  // transform pointcloud
  sensor_msgs::PointCloud2 cloud_trans;
  pcl_ros::transformPointCloud("world", *cloud_ptr, cloud_trans, tf_listener_);

  // convert to pcl
  PointCloud::Ptr pcl_cloud_ptr = PointCloud().makeShared();
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(cloud_trans, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, *pcl_cloud_ptr);
  // pcl_conversions::fromROSMsg(cloud_trans, pcl_cloud_ptr);  // maybe this works in kinetic?

  // push to mapper
  global_mapper_.PushPointCloud(pcl_cloud_ptr);
}

void GlobalMapperRos::Run() {
  GetParams();
  InitSubscribers();
  InitPublishers();

  // start mapping thread
  global_mapper_.Run();

  // handle ros callbacks
  ros::spin();
}

}  // namespace global_mapper
