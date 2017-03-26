// Copyright 2017 Massachusetts Institute of Technology
#pragma once

#include <csignal>
#include <memory>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "message_filters/subscriber.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_ros/point_cloud.h"
#include "visualization_msgs/MarkerArray.h"
#include "nav_msgs/OccupancyGrid.h"

#include "global_mapper/global_mapper.h"

namespace global_mapper {

class GlobalMapperRos {
 public:
  explicit GlobalMapperRos(volatile std::sig_atomic_t* stop_signal_ptr);

  void Run();

  volatile std::sig_atomic_t* stop_signal_ptr_;

 private:
  void GetParams(GlobalMapperParams& global_mapper_params);
  void InitSubscribers();
  void InitPublishers();
  void PopulatePixelMapMsg(nav_msgs::OccupancyGrid* occupancy_grid);
  void PopulateVoxelMapMsg(visualization_msgs::MarkerArray* marker_array);
  void PublishMap(const ros::TimerEvent& event);
  std::vector<double> GrayscaleToRGBJet(double v, double vmin, double vmax);

  // callbacks
  void PointCloudCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud_ptr);

  // publishers
  ros::Publisher pixel_map_pub_;
  ros::Publisher voxel_map_pub_;
  ros::Timer map_pub_timer_;
  ros::Publisher marker_pub_;

  // subscribers
  ros::Subscriber point_cloud_sub_;

  // params
  GlobalMapperParams params_;
  bool publish_pixel_map_;
  bool publish_voxel_map_;

  std::unique_ptr<GlobalMapper> global_mapper_ptr_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::Buffer tf_buffer_;
};
}  // namespace global_mapper
