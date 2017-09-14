// Copyright 2017 Massachusetts Institute of Technology

#include <csignal>
#include <memory>
#include <utility>
#include <algorithm>

#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>

#include <fla_utils/param_utils.h>

#include "global_mapper_ros/global_mapper_ros.h"
#include "global_mapper/global_mapper.h"
#include "global_mapper/params.h"

namespace global_mapper {

GlobalMapperRos::GlobalMapperRos(volatile std::sig_atomic_t* stop_signal_ptr)
  : stop_signal_ptr_(stop_signal_ptr),
    publish_voxel_map_(false),
    nh_(),
    pnh_("~"),
    tf_listener_(tf_buffer_) {
  tf_buffer_.setUsingDedicatedThread(true);

  GetParams();
  InitSubscribers();
  InitPublishers();
}

void GlobalMapperRos::GetParams() {
  fla_utils::SafeGetParam(pnh_, "map_frame", params_.map_frame);

  // pixel map params
  std::vector<double> pixel_xy_min(2, 0.0);
  fla_utils::SafeGetParam(pnh_, "pixel_map/xy_min", pixel_xy_min);
  std::copy(pixel_xy_min.begin(), pixel_xy_min.end(), params_.pixel_xy_min);

  std::vector<double> pixel_xy_max(2, 0.0);
  fla_utils::SafeGetParam(pnh_, "pixel_map/xy_max", pixel_xy_max);
  std::copy(pixel_xy_max.begin(), pixel_xy_max.end(), params_.pixel_xy_max);

  fla_utils::SafeGetParam(pnh_, "pixel_map/resolution", params_.pixel_resolution);
  fla_utils::SafeGetParam(pnh_, "pixel_map/init_value", params_.pixel_init_value);
  fla_utils::SafeGetParam(pnh_, "pixel_map/bound_min", params_.pixel_bound_min);
  fla_utils::SafeGetParam(pnh_, "pixel_map/bound_max", params_.pixel_bound_max);
  fla_utils::SafeGetParam(pnh_, "pixel_map/min_z_abs", params_.pixel_min_z_abs);
  fla_utils::SafeGetParam(pnh_, "pixel_map/max_z_abs", params_.pixel_max_z_abs);
  fla_utils::SafeGetParam(pnh_, "pixel_map/use_rel_flatten", params_.pixel_use_rel_flatten);
  fla_utils::SafeGetParam(pnh_, "pixel_map/min_z_rel", params_.pixel_min_z_rel);
  fla_utils::SafeGetParam(pnh_, "pixel_map/max_z_rel", params_.pixel_max_z_rel);
  fla_utils::SafeGetParam(pnh_, "pixel_map/publish_map", publish_pixel_map_);

  // voxel map params
  std::vector<double> voxel_xyz_min(3, 0.0);
  fla_utils::SafeGetParam(pnh_, "voxel_map/xyz_min", voxel_xyz_min);
  std::copy(voxel_xyz_min.begin(), voxel_xyz_min.end(), params_.voxel_xyz_min);

  std::vector<double> voxel_xyz_max(3, 0.0);
  fla_utils::SafeGetParam(pnh_, "voxel_map/xyz_max", voxel_xyz_max);
  std::copy(voxel_xyz_max.begin(), voxel_xyz_max.end(), params_.voxel_xyz_max);

  fla_utils::SafeGetParam(pnh_, "voxel_map/resolution", params_.voxel_resolution);

  fla_utils::SafeGetParam(pnh_, "voxel_map/init_value", params_.voxel_init_value);
  fla_utils::SafeGetParam(pnh_, "voxel_map/bound_min", params_.voxel_bound_min);
  fla_utils::SafeGetParam(pnh_, "voxel_map/bound_max", params_.voxel_bound_max);
  fla_utils::SafeGetParam(pnh_, "voxel_map/min_z_abs", params_.voxel_min_z_abs);
  fla_utils::SafeGetParam(pnh_, "voxel_map/max_z_abs", params_.voxel_max_z_abs);
  fla_utils::SafeGetParam(pnh_, "voxel_map/use_rel_cropping", params_.voxel_use_rel_cropping);
  fla_utils::SafeGetParam(pnh_, "voxel_map/min_z_rel", params_.voxel_min_z_rel);
  fla_utils::SafeGetParam(pnh_, "voxel_map/max_z_rel", params_.voxel_max_z_rel);
  fla_utils::SafeGetParam(pnh_, "voxel_map/hit_inc", params_.voxel_hit_inc);
  fla_utils::SafeGetParam(pnh_, "voxel_map/miss_inc", params_.voxel_miss_inc);
  fla_utils::SafeGetParam(pnh_, "voxel_map/publish_map", publish_voxel_map_);
}

void GlobalMapperRos::InitSubscribers() {
  point_cloud_sub_ = pnh_.subscribe("cloud_topic", 10, &GlobalMapperRos::PointCloudCallback, this);
}

void GlobalMapperRos::InitPublishers() {
  pixel_map_pub_ = pnh_.advertise<nav_msgs::OccupancyGrid>("pixel_map_topic", 1);
  voxel_map_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("voxel_map_topic", 1);
  pointcloud_pub_ = pnh_.advertise<sensor_msgs::PointCloud2>("pointcloud_topic", 1);
  map_pub_timer_ = nh_.createTimer(ros::Duration(0.2), &GlobalMapperRos::PublishMap, this);
}

/*
  Return a RGB colour value given a scalar v in the range [vmin,vmax]
  In this case each colour component ranges from 0 (no contribution) to
  1 (fully saturated). The colour is clipped at the end of the scales if v is outside
  the range [vmin,vmax]
*/
void GlobalMapperRos::GrayscaleToRGBJet(double v, double vmin, double vmax, std::vector<double>* rgb) {
  double dv;

  if (v < vmin)
    v = vmin;
  if (v > vmax)
    v = vmax;
  dv = vmax - vmin;

  if (v < (vmin + 0.25 * dv)) {
    rgb->at(0) = 0;
    rgb->at(1) = 4 * (v - vmin) / dv;
  } else if (v < (vmin + 0.5 * dv)) {
    rgb->at(0) = 0;
    rgb->at(2) = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
  } else if (v < (vmin + 0.75 * dv)) {
    rgb->at(0) = 4 * (v - vmin - 0.5 * dv) / dv;
    rgb->at(2) = 0;
  } else {
    rgb->at(1) = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
    rgb->at(2) = 0;
  }
}

void GlobalMapperRos::PopulatePointCloudMsg(sensor_msgs::PointCloud2* pointcloud) {
  // check for bad input
  if (pointcloud == nullptr) {
    return;
  }

  geometry_msgs::TransformStamped transform_stamped;
  Eigen::Vector3d transform;
  try {
    transform_stamped = tf_buffer_.lookupTransform("world", "body",
                                                   ros::Time(0), ros::Duration(1.0));
    transform(0) = transform_stamped.transform.translation.x;
    transform(1) = transform_stamped.transform.translation.y;
    transform(2) = transform_stamped.transform.translation.z;
  } catch (tf2::TransformException &ex) {
    ROS_WARN("[world_database_master_ros] OnGetTransform failed with %s", ex.what());

    transform(0) = std::numeric_limits<double>::quiet_NaN();
    transform(1) = std::numeric_limits<double>::quiet_NaN();
    transform(2) = std::numeric_limits<double>::quiet_NaN();
  }

  // get occuppied voxels
  std::vector<int> occ_inds;

  int min_ixyz[3];
  int max_ixyz[3];

  const double subgrid_half_size[3] = {20,20,3};

  const double min_xyz[3] = {transform[0] - subgrid_half_size[0], 
                       transform[1] - subgrid_half_size[1],
                       transform[2] - subgrid_half_size[2]};

  const double max_xyz[3] = {transform[0] + subgrid_half_size[0], 
                       transform[1] + subgrid_half_size[1],
                       transform[2] + subgrid_half_size[2]};


  global_mapper_ptr_->voxel_map_ptr_->WorldToGrid(min_xyz, min_ixyz);
  global_mapper_ptr_->voxel_map_ptr_->WorldToGrid(max_xyz, max_ixyz);

  // int voxel_index = 0;
  // for (int x = min_ixyz[0]; x < max_ixyz[0]; x++) {
  //   for (int y = min_ixyz[1]; y < max_ixyz[1]; y++) {
  //     for (int z = min_ixyz[2]; z < max_ixyz[2]; z++) {
  //       int ixyz[3] = {x, y, z};
  //       if(global_mapper_ptr_->voxel_map_ptr_->ReadValue(ixyz) > 0.6) {
  //         voxel_index = global_mapper_ptr_->voxel_map_ptr_->GetIndex(ixyz);
  //         occ_inds.push_back(voxel_index);
  //       }
  //     }
  //   }
  // }

  // double xyz[3] = {0.0};
  // pcl::PointCloud<pcl::PointXYZ> cloud;
  // for (int i = 0; i < occ_inds.size(); ++i) {
  //   global_mapper_ptr_->voxel_map_ptr_->IndexToWorld(occ_inds[i], xyz);
  //   cloud.push_back(pcl::PointXYZ(xyz[0], xyz[1], xyz[2]));
  // }

  // pcl::toROSMsg(cloud, *pointcloud);
  // pointcloud->header.frame_id = "world";
  // pointcloud->header.stamp = ros::Time::now();
}

void GlobalMapperRos::PopulatePixelMapMsg(nav_msgs::OccupancyGrid* occupancy_grid) {
  // // check for bad input
  // if (occupancy_grid == nullptr) {
  //   return;
  // }

  // // header
  // occupancy_grid->header.frame_id = params_.map_frame;
  // occupancy_grid->header.stamp = ros::Time::now();

  // // metadata
  // occupancy_grid->info.resolution = global_mapper_ptr_->pixel_map_ptr_->metersPerPixel;
  // occupancy_grid->info.width = global_mapper_ptr_->pixel_map_ptr_->dimensions[0];
  // occupancy_grid->info.height = global_mapper_ptr_->pixel_map_ptr_->dimensions[1];
  // occupancy_grid->info.origin.position.x = 0.0;
  // occupancy_grid->info.origin.position.y = 0.0;
  // occupancy_grid->info.origin.position.z = 0.0;

  // // data
  // float occ_value = 0.0;
  // double xy[2] = {0.0};
  // occupancy_grid->data.resize(global_mapper_ptr_->pixel_map_ptr_->num_cells);
  // for (int i = 0; i < global_mapper_ptr_->pixel_map_ptr_->num_cells; ++i) {
  //   // get xy location
  //   global_mapper_ptr_->pixel_map_ptr_->IndexToWorld(i, xy);

  //   // get pixel value and scale
  //   occ_value = global_mapper_ptr_->pixel_map_ptr_->ReadValue(xy);
  //   occ_value *= 100.0;
  //   occupancy_grid->data[i] = static_cast<uint8_t>(occ_value + 0.5);
  // }
}

void GlobalMapperRos::PublishMap(const ros::TimerEvent& event) {
  // lock
  // std::lock_guard<std::mutex> map_lock(global_mapper_ptr_->map_mutex());

  // voxel map
  if (publish_voxel_map_) {
    // double start_time = ros::Time::now().toSec();
    // visualization_msgs::MarkerArray marker_array;
    // PopulateVoxelMapMsg(&marker_array);
    // std::cout << "(global_mapper_ros) PopulateVoxelMapMsg took " << ros::Time::now().toSec() - start_time << " seconds" << std::endl;
    // voxel_map_pub_.publish(marker_array);

    double start_time = ros::Time::now().toSec();
    sensor_msgs::PointCloud2 pointcloud_msg;
    PopulatePointCloudMsg(&pointcloud_msg);
    std::cout << "(global_mapper_ros) PopulatePointCloudMsg took " << ros::Time::now().toSec() - start_time << " seconds" << std::endl;
    pointcloud_pub_.publish(pointcloud_msg);
  }

  // pixel_map
  if (publish_pixel_map_) {
    nav_msgs::OccupancyGrid occupancy_grid;
    PopulatePixelMapMsg(&occupancy_grid);
    pixel_map_pub_.publish(occupancy_grid);
  }
}

void GlobalMapperRos::PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_ptr) {
  // get transform
  const std::string target_frame = params_.map_frame;
  geometry_msgs::TransformStamped transform_stamped;
  // try to get correct transform
  try {
    transform_stamped = tf_buffer_.lookupTransform(target_frame, cloud_ptr->header.frame_id,
                                                   ros::Time(cloud_ptr->header.stamp),
                                                   ros::Duration(0.02));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return;
  }

  sensor_msgs::PointCloud2 cloud_out;
  tf2::doTransform(*cloud_ptr, cloud_out, transform_stamped);

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(cloud_out, cloud);
  
  // push to mapper
  // static bool virgin = true;
  // if (virgin) {
    global_mapper_ptr_->PushPointCloud(cloud.makeShared());
    // virgin = false;
  // }
}

void GlobalMapperRos::Run() {
  // start mapping thread
  global_mapper_ptr_ = std::unique_ptr<GlobalMapper>(new GlobalMapper(stop_signal_ptr_, params_));
  global_mapper_ptr_->Run();

  // handle ros callbacks
  ros::spin();
}

}  // namespace global_mapper
