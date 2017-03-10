// Copyright 2017 Massachusetts Institute of Technology

#include <csignal>

#include "ros/ros.h"
#include "nodelet/nodelet.h"

#include "global_mapper_ros/global_mapper_ros.h"

namespace global_mapper {

class GlobalMapperNodelet : public nodelet::Nodelet {
 public:
  GlobalMapperNodelet() = default;
  ~GlobalMapperNodelet() = default;

  GlobalMapperNodelet(const GlobalMapperNodelet& rhs) = delete;
  GlobalMapperNodelet& operator=(const GlobalMapperNodelet& rhs) = delete;

  GlobalMapperNodelet(GlobalMapperNodelet&& rhs) = delete;
  GlobalMapperNodelet& operator=(GlobalMapperNodelet&& rhs) = delete;

  /**
   * We override the default ROS SIGINT handler to set a global variable which
   * tells the other threads to stop.
   */
  static void signal_handler(int signal) {
    printf("(svis_ros) SIGINT received\n");

    // Tell other threads to stop.
    GlobalMapperNodelet::stop_signal_ = 1;

    // Tell ROS to shutdown nodes.
    ros::shutdown();

    return;
  }

  // sigint
  static volatile std::sig_atomic_t stop_signal_;

  /**
   * \brief Nodelet initialization.
   *
   * Subclasses of nodelet::Nodelet need to override this virtual method.
   * It takes the place of the nodelet constructor.
   */
  virtual void onInit() {
    // Install signal handler.
    std::signal(SIGINT, signal_handler);

    // GlobalMapperRos global_mapper_ros;
    // NODELET_INFO("Starting loop");
    // global_mapper_ros.Run(stop_signal_);

    return;
  }
};

volatile std::sig_atomic_t stop_signal_ = 0;

}  // namespace global_mapper

// Export as a plugin.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(global_mapper::GlobalMapperNodelet, nodelet::Nodelet)
