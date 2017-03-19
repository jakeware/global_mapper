// Copyright 2017 Massachusetts Institute of Technology

#include <csignal>
#include <memory>

#include "ros/ros.h"
#include "pcl_ros/transforms.h"
#include "pcl/conversions.h"
#include "visualization_msgs/MarkerArray.h"

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
  marker_pub_ = nh_.advertise<visualization_msgs::Marker>("marker_topic", 1);
}

void GlobalMapperRos::PublishMap(const ros::TimerEvent& event) {
  // lock
  std::lock_guard<std::mutex> lock(global_mapper_.map_mutex_);

  // TODO(jakeware): Break all of this into a map class
  const double x_origin = 0.0;
  const double y_origin = 0.0;
  const double z_origin = 0.0;
  const double x_res = 1.0;
  const double y_res = 1.0;
  const double z_res = 1.0;
  const uint32_t x_size = global_mapper_.ixyz_max_[0];
  const uint32_t y_size = global_mapper_.ixyz_max_[1];
  const uint32_t z_size = global_mapper_.ixyz_max_[2];

  // get occuppied voxels
  std::vector<std::vector<double> > voxel_vec;
  uint32_t num_voxels = 0;
  int ixyz[3] = {0};
  std::vector<double> voxel(3, 0.0);
  for (int i = 0; i < global_mapper_.global_map_.size(); i++) {
    if (global_mapper_.global_map_[i] > 0.0) {
      // TODO(jakeware): Add index to world conversion
      global_mapper_.IndToCoord(i, ixyz);
      voxel[0] = x_origin + (static_cast<double>(ixyz[0]) + 0.5) * x_res;
      voxel[1] = y_origin + (static_cast<double>(ixyz[1]) + 0.5) * y_res;
      voxel[2] = z_origin + (static_cast<double>(ixyz[2]) + 0.5) * z_res;
      voxel_vec.push_back(voxel);

      ++num_voxels;
    }
  }

  // populate marker message
  visualization_msgs::Marker m;
  m.header.frame_id = "world";
  m.header.stamp = ros::Time::now();
  m.ns = "global_mapper";
  m.id = 0;
  m.type = visualization_msgs::Marker::CUBE_LIST;
  m.action = visualization_msgs::Marker::ADD;
  m.pose.orientation.w = 1.0;
  m.scale.x = x_res;
  m.scale.y = y_res;
  m.scale.z = z_res;
  m.color.r = 1.0f;
  m.color.g = 0.0f;
  m.color.b = 0.0f;
  m.color.a = 1.0f;
  m.points.resize(num_voxels);
  for (uint32_t i = 0; i < num_voxels; ++i) {
    m.points[i].x = voxel_vec[i][0];
    m.points[i].y = voxel_vec[i][1];
    m.points[i].z = voxel_vec[i][2];
  }

  // publish
  marker_pub_.publish(m);
}

// void GlobalMapperRos::PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_ptr) {
void GlobalMapperRos::PointCloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_ptr) {
  // get transform
  const std::string target_frame = "world";
  geometry_msgs::TransformStamped transform_stamped;
  // try to get correct transform
  try {
    // TODO(jakeware): FIX THIS! WHY THE FUCK DOES THIS NOT WORK?
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
