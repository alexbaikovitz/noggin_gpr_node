#ifndef NOGGIN_GPR_ROS_H
#define NOGGIN_GPR_ROS_H

#include <ros/ros.h>
#include <vector>
#include <string>

namespace lgpr::noggin {

struct NogginGprConfig {
  int device_id;

  int baud_rate;

  int sampling_frequency;

  bool filter;

  int points;

  int point_offset;

  std::string output_trace_topic;

  bool bypass_device_startup;
};

class NogginGprRos {
  public:
    NogginGprConfig noggin_config_;

    NogginGprRos();
    ~NogginGprRos() = default;

    // Method to publish trace from GPR.
    void PublishTrace(const ros::Time& time, const std::vector<int16_t>& trace);

  private:
    ros::NodeHandle nh_;
    ros::Publisher trace_pub_;
    int seq_;
};

}  // namespace lgpr::noggin

#endif