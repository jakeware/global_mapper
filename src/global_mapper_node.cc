// Copyright 2017 Massachusetts Institute of Technology

#include <csignal>

#include "ros/ros.h"

#include "global_mapper_ros/global_mapper_ros.h"

namespace global_mapper {

volatile std::sig_atomic_t stop_signal = 0;

/**
 * We override the default ROS SIGINT handler to set a global variable which
 * tells the other threads to stop.
 */
static void signal_handler(int signal) {
  printf("(global_mapper) SIGINT received\n");

  // Tell other threads to stop.
  stop_signal = 1;

  // Tell ROS to shutdown nodes.
  ros::shutdown();

  return;
}

}  // namespace global_mapper

int main(int argc, char **argv) {
  ros::init(argc, argv, "global_mapper");

  // Install signal handler.
  std::signal(SIGINT, global_mapper::signal_handler);

  global_mapper::GlobalMapperRos global_mapper_ros(&global_mapper::stop_signal);
  ROS_INFO("Starting loop");
  global_mapper_ros.Run();

  return 0;
}
