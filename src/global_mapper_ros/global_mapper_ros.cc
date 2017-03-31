// Copyright 2017 Massachusetts Institute of Technology

#include <csignal>
#include <memory>
#include <utility>
#include <algorithm>

#include <ros/ros.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>

#include <fla_utils/param_utils.h>

#include "global_mapper_ros/global_mapper_ros.h"
#include "global_mapper/global_mapper.h"
#include "global_mapper/global_mapper_params.h"

namespace global_mapper {

GlobalMapperRos::GlobalMapperRos(volatile std::sig_atomic_t* stop_signal_ptr)
  : stop_signal_ptr_(stop_signal_ptr),
    publish_voxel_map_(false),
    nh_(),
    pnh_("~"),
    tf_listener_(tf_buffer_) {
  tf_buffer_.setUsingDedicatedThread(true);
}

void GlobalMapperRos::GetParams(Params* params) {
  // pixel map params
  std::vector<double> pixel_xy_min(2, 0.0);
  fla_utils::SafeGetParam(pnh_, "pixel_map/xy_min", pixel_xy_min);
  std::copy(pixel_xy_min.begin(), pixel_xy_min.end(), params->pixel_xy_min_);

  std::vector<double> pixel_xy_max(2, 0.0);
  fla_utils::SafeGetParam(pnh_, "pixel_map/xy_max", pixel_xy_max);
  std::copy(pixel_xy_max.begin(), pixel_xy_max.end(), params->pixel_xy_max_);

  fla_utils::SafeGetParam(pnh_, "pixel_map/resolution", params->pixel_resolution_);
  fla_utils::SafeGetParam(pnh_, "pixel_map/init_value", params->pixel_init_value_);
  fla_utils::SafeGetParam(pnh_, "pixel_map/bound_min", params->pixel_bound_min_);
  fla_utils::SafeGetParam(pnh_, "pixel_map/bound_max", params->pixel_bound_max_);
  fla_utils::SafeGetParam(pnh_, "pixel_map/min_z_abs", params->pixel_min_z_abs_);
  fla_utils::SafeGetParam(pnh_, "pixel_map/max_z_abs", params->pixel_max_z_abs_);
  fla_utils::SafeGetParam(pnh_, "pixel_map/use_rel_flatten", params->pixel_use_rel_flatten_);
  fla_utils::SafeGetParam(pnh_, "pixel_map/min_z_rel", params->pixel_min_z_rel_);
  fla_utils::SafeGetParam(pnh_, "pixel_map/max_z_rel", params->pixel_max_z_rel_);
  fla_utils::SafeGetParam(pnh_, "pixel_map/publish_map", publish_pixel_map_);

  // voxel map params
  std::vector<double> voxel_xyz_min(3, 0.0);
  fla_utils::SafeGetParam(pnh_, "voxel_map/xyz_min", voxel_xyz_min);
  std::copy(voxel_xyz_min.begin(), voxel_xyz_min.end(), params->voxel_xyz_min_);

  std::vector<double> voxel_xyz_max(3, 0.0);
  fla_utils::SafeGetParam(pnh_, "voxel_map/xyz_max", voxel_xyz_max);
  std::copy(voxel_xyz_max.begin(), voxel_xyz_max.end(), params->voxel_xyz_max_);

  std::vector<double> voxel_resolution(3, 0.0);
  fla_utils::SafeGetParam(pnh_, "voxel_map/resolution", voxel_resolution);
  std::copy(voxel_resolution.begin(), voxel_resolution.end(), params->voxel_resolution_);

  fla_utils::SafeGetParam(pnh_, "voxel_map/init_value", params->voxel_init_value_);
  fla_utils::SafeGetParam(pnh_, "voxel_map/bound_min", params->voxel_bound_min_);
  fla_utils::SafeGetParam(pnh_, "voxel_map/bound_max", params->voxel_bound_max_);
  fla_utils::SafeGetParam(pnh_, "voxel_map/min_z_abs", params->voxel_min_z_abs_);
  fla_utils::SafeGetParam(pnh_, "voxel_map/max_z_abs", params->voxel_max_z_abs_);
  fla_utils::SafeGetParam(pnh_, "voxel_map/use_rel_cropping", params->voxel_use_rel_cropping_);
  fla_utils::SafeGetParam(pnh_, "voxel_map/min_z_rel", params->voxel_min_z_rel_);
  fla_utils::SafeGetParam(pnh_, "voxel_map/max_z_rel", params->voxel_max_z_rel_);
  fla_utils::SafeGetParam(pnh_, "voxel_map/hit_inc", params->voxel_hit_inc_);
  fla_utils::SafeGetParam(pnh_, "voxel_map/miss_inc", params->voxel_miss_inc_);
  fla_utils::SafeGetParam(pnh_, "voxel_map/publish_map", publish_voxel_map_);
}

void GlobalMapperRos::InitSubscribers() {
  point_cloud_sub_ = nh_.subscribe<pcl::PointCloud<pcl::PointXYZ> >("cloud_topic", 10, &GlobalMapperRos::PointCloudCallback, this);
}

void GlobalMapperRos::InitPublishers() {
  pixel_map_pub_ = pnh_.advertise<nav_msgs::OccupancyGrid>("pixel_map_topic", 1);
  voxel_map_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("voxel_map_topic", 1);
  map_pub_timer_ = nh_.createTimer(ros::Duration(1.0), &GlobalMapperRos::PublishMap, this);
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

void GlobalMapperRos::PopulateVoxelMapMsg(visualization_msgs::MarkerArray* marker_array) {
  // check for bad input
  if (marker_array == nullptr) {
    return;
  }

  // get occuppied voxels
  std::vector<std::unique_ptr<double[]> > voxel_vec;

  double xyz[3] = {0.0};
  for (int i = 0; i < global_mapper_ptr_->voxel_map_ptr_->num_cells; i++) {
    global_mapper_ptr_->voxel_map_ptr_->indToLoc(i, xyz);

    if (global_mapper_ptr_->voxel_map_ptr_->readValue(xyz) > 0.6) {
      std::unique_ptr<double[]> voxel_ptr(new double[3]);
      voxel_ptr[0] = xyz[0];
      voxel_ptr[1] = xyz[1];
      voxel_ptr[2] = xyz[2];
      voxel_vec.push_back(std::move(voxel_ptr));
    }
  }
  int num_voxels = voxel_vec.size();
  ROS_INFO("num_voxels: %u", num_voxels);

  visualization_msgs::Marker marker;
  std::vector<double> rgb(3, 1.0);
  for (int i = 0; i < num_voxels; ++i) {
    // create voxel marker
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.id = i;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    // orientation
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // // scale
    marker.scale.x = global_mapper_ptr_->voxel_map_ptr_->metersPerPixel[0];
    marker.scale.y = global_mapper_ptr_->voxel_map_ptr_->metersPerPixel[1];
    marker.scale.z = global_mapper_ptr_->voxel_map_ptr_->metersPerPixel[2];

    // position
    marker.pose.position.x = voxel_vec[i][0];
    marker.pose.position.y = voxel_vec[i][1];
    marker.pose.position.z = voxel_vec[i][2];

    // color
    GrayscaleToRGBJet(marker.pose.position.z,
                      global_mapper_ptr_->voxel_map_ptr_->xyz0[2],
                      global_mapper_ptr_->voxel_map_ptr_->xyz1[2],
                      &rgb);
    marker.color.r = static_cast<float>(rgb[0]);
    marker.color.g = static_cast<float>(rgb[1]);
    marker.color.b = static_cast<float>(rgb[2]);
    marker.color.a = 1.0f;

    // add voxel marker to marker array
    marker_array->markers.push_back(marker);
  }
}

void GlobalMapperRos::PopulatePixelMapMsg(nav_msgs::OccupancyGrid* occupancy_grid) {
  // check for bad input
  if (occupancy_grid == nullptr) {
    return;
  }

  // header
  occupancy_grid->header.frame_id = "world";
  occupancy_grid->header.stamp = ros::Time::now();

  // metadata
  occupancy_grid->info.resolution = global_mapper_ptr_->pixel_map_ptr_->metersPerPixel;
  occupancy_grid->info.width = global_mapper_ptr_->pixel_map_ptr_->dimensions[0];
  occupancy_grid->info.height = global_mapper_ptr_->pixel_map_ptr_->dimensions[1];
  occupancy_grid->info.origin.position.x = 0.0;
  occupancy_grid->info.origin.position.y = 0.0;
  occupancy_grid->info.origin.position.z = 0.0;

  // data
  float occ_value = 0.0;
  double xy[2] = {0.0};
  occupancy_grid->data.resize(global_mapper_ptr_->pixel_map_ptr_->num_cells);
  for (int i = 0; i < global_mapper_ptr_->pixel_map_ptr_->num_cells; ++i) {
    // get xy location
    global_mapper_ptr_->pixel_map_ptr_->indToLoc(i, xy);

    // get pixel value and scale
    occ_value = global_mapper_ptr_->pixel_map_ptr_->readValue(xy);

    if (occ_value < 0.0 || occ_value > 1.0) {
      ROS_INFO("occ_value: %0.2f", occ_value);
    }
    occupancy_grid->data[i] = static_cast<uint8_t>(occ_value * 100.0);
  }
}

void GlobalMapperRos::PublishMap(const ros::TimerEvent& event) {
  // lock
  std::unique_lock<std::mutex> map_lock = global_mapper_ptr_->MapLock();

  // voxel map
  if (publish_voxel_map_) {
    visualization_msgs::MarkerArray marker_array;
    PopulateVoxelMapMsg(&marker_array);
    voxel_map_pub_.publish(marker_array);
  }

  // pixel_map
  if (publish_pixel_map_) {
    nav_msgs::OccupancyGrid occupancy_grid;
    PopulatePixelMapMsg(&occupancy_grid);
    pixel_map_pub_.publish(occupancy_grid);
  }
}

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

  // set origin of pointcloud
  // NOTE(jakeware): feel like there must be a better way to do this.
  cloud_trans_ptr->sensor_origin_[0] = transform_stamped.transform.translation.x;
  cloud_trans_ptr->sensor_origin_[1] = transform_stamped.transform.translation.y;
  cloud_trans_ptr->sensor_origin_[2] = transform_stamped.transform.translation.z;

  // push to mapper
  global_mapper_ptr_->PushPointCloud(cloud_trans_ptr);
}

void GlobalMapperRos::Run() {
  Params params;
  GetParams(&params);
  InitSubscribers();
  InitPublishers();

  // start mapping thread
  global_mapper_ptr_ = std::unique_ptr<GlobalMapper>(new GlobalMapper(stop_signal_ptr_, params));
  global_mapper_ptr_->Run();

  // handle ros callbacks
  ros::spin();
}

}  // namespace global_mapper
