#include "noggin_gpr_node/noggin_gpr_ros.h"
#include "noggin_gpr_node/StampedWaveform.h"

#include <ros/ros.h>

namespace lgpr::noggin {

namespace {

constexpr int kTraceQueue = 20;

}  // namespace

NogginGprRos::NogginGprRos() : nh_("~") {
  // Populate the noggin configuration structure.
  nh_.getParam("/device_id", noggin_config_.device_id);
  nh_.getParam("/baud_rate", noggin_config_.baud_rate);
  nh_.getParam("/sampling_frequency", noggin_config_.sampling_frequency);
  nh_.getParam("/filter", noggin_config_.filter);
  nh_.getParam("/points", noggin_config_.points);
  nh_.getParam("/point_offset", noggin_config_.point_offset);
  nh_.getParam("/output_trace_topic", noggin_config_.output_trace_topic);
  nh_.getParam("/bypass_device_startup", noggin_config_.bypass_device_startup);

  // ROS publisher for StampedWaveform message.
  trace_pub_ = nh_.advertise<noggin_gpr_node::StampedWaveform>(
    noggin_config_.output_trace_topic, kTraceQueue);
}

void NogginGprRos::PublishTrace(const ros::Time& time, 
                                const std::vector<int16_t>& trace) {
  noggin_gpr_node::StampedWaveform out;
  out.header.stamp = time;
  out.header.seq = seq_;
  seq_++;
  out.trace = trace;
  trace_pub_.publish(out);
}

}  // namespace lgpr::noggin