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
  point_cloud_sub_.subscribe(nh_, "cloud_topic", 10);
  tf_filter_ = std::make_shared<tf::MessageFilter<sensor_msgs::PointCloud2> >(point_cloud_sub_, tf_listener_, "world", 10);
  tf_filter_->registerCallback(boost::bind(&GlobalMapperRos::PointCloudCallback, this, _1));
}

void GlobalMapperRos::InitPublishers() {
  map_pub_ = pnh_.advertise<PointCloud>("map_topic", 1);
  map_pub_timer_ = nh_.createTimer(ros::Duration(1.0), &GlobalMapperRos::PublishMap, this);
}

void GlobalMapperRos::PublishMap(const ros::TimerEvent& event) {
  // lock
  std::lock_guard<std::mutex> lock(global_mapper_.mutex_);

  // populate
  PointCloud map;
  map.header.frame_id = "world";
  map.height = 1;
  map.width = global_mapper_.global_map_.size();
  int ixyz[3] = {0};
  for (int i = 0; i < global_mapper_.global_map_.size(); i++) {
    if (global_mapper_.global_map_[i] > 0.0) {
      ROS_INFO("check1");
      global_mapper_.IndToCoord(i, ixyz);
      map.points.push_back(pcl::PointXYZ(ixyz[0], ixyz[1], ixyz[2]));
    }
  }

  // publish
  map_pub_.publish(map);
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
