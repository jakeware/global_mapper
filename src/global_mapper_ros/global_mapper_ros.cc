// Copyright 2017 Massachusetts Institute of Technology

#include <csignal>
#include <memory>
#include <string>

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
  tf_listener_(tf_buffer_) {
}

GlobalMapperRos::~GlobalMapperRos() {
  // nothing
}

void GlobalMapperRos::GetParams() {
  fla_utils::SafeGetParam(pnh_, "test_param", test_param_);
}

void GlobalMapperRos::InitSubscribers() {
  point_cloud_sub_ = nh_.subscribe<pcl::PointCloud<pcl::PointXYZ> >("cloud_topic", 10, &GlobalMapperRos::PointCloudCallback, this);
}

void GlobalMapperRos::InitPublishers() {
  map_pub_ = pnh_.advertise<PointCloud>("map_topic", 1);
  map_pub_timer_ = nh_.createTimer(ros::Duration(1.0), &GlobalMapperRos::PublishMap, this);
}

void GlobalMapperRos::PublishMap(const ros::TimerEvent& event) {
  // lock
  std::lock_guard<std::mutex> lock(global_mapper_.map_mutex_);

  // populate
  PointCloud map;
  map.header.frame_id = "world";
  map.height = 1;

  int ixyz[3] = {0};
  int count = 0;
  for (int i = 0; i < global_mapper_.global_map_.size(); i++) {
    if (global_mapper_.global_map_[i] > 0.0) {
      count++;
      global_mapper_.IndToCoord(i, ixyz);
      map.points.push_back(pcl::PointXYZ(ixyz[0], ixyz[1], ixyz[2]));
    }
  }
  map.width = map.points.size();
  ROS_INFO("map points: %i", count);

  // publish
  map_pub_.publish(map);
}

// void GlobalMapperRos::PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_ptr) {
void GlobalMapperRos::PointCloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_ptr) {
  // get transform
  const std::string target_frame = "world";
  geometry_msgs::TransformStamped transform_stamped;
  // try to get correct transform
  try {
    // transform_stamped = tf_buffer_.lookupTransform(target_frame, cloud_ptr->header.frame_id,
    //                                                ros::Time(cloud_ptr->header.stamp),
    //                                                ros::Duration(0.1));
    transform_stamped = tf_buffer_.lookupTransform(target_frame,
                                                   cloud_ptr->header.frame_id,
                                                   ros::Time(0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());

    // try again with latest transform
    try {
      transform_stamped = tf_buffer_.lookupTransform(target_frame,
                                                     cloud_ptr->header.frame_id,
                                                     ros::Time(0));
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      return;
    }
  }

  // convert transform
  tf::Transform transform;
  tf::transformMsgToTF(transform_stamped.transform, transform);

  // transform pointcloud
  pcl::PointCloud<pcl::PointXYZ> cloud_trans;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trans_ptr = cloud_trans.makeShared();
  pcl_ros::transformPointCloud(*cloud_ptr, *cloud_trans_ptr, transform);

  // push to mapper
  global_mapper_.PushPointCloud(cloud_trans_ptr);
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
