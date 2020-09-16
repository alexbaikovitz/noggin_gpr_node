#include "noggin_gpr_node/noggin_gpr.h"

#include "gflags/gflags.h"
#include <glog/logging.h>
#include <ros/ros.h>

int main (int argc, char** argv) {
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[1]);
  ROS_INFO_STREAM("Launched Noggin GPR Node\n");

  ros::init(argc, argv, "noggin_gpr_node");
  auto noggin_device = lgpr::noggin::NogginGpr();

  ros::Rate rate(noggin_device.GetSamplingFrequency());
  while (ros::ok()) {
    if (noggin_device.ready_to_sample_) {
      noggin_device.RequestTrace();
    }
    rate.sleep();
  }
  return EXIT_SUCCESS;
}