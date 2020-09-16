#include "noggin_gpr_node/noggin_gpr.h"

#include <glog/logging.h>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <serial/serial.h>
#include <absl/strings/match.h>
#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "absl/base/thread_annotations.h"
#include "ros/time.h"


namespace lgpr::noggin {

namespace {

constexpr char kRequestTraceCmd[] = "T\r";
constexpr char kOnCmd[] = "XON\r";
constexpr char kReportCmd[] = "R\r";
constexpr char kSerialPortPath[] = "/dev/ttyUSB0";
constexpr char kOnStateIndicator[] = "TPU";
constexpr char kTracePrefix[] = "T01";
// TODO(abaikovitz) Clean up serial comms.
constexpr int kAdditionalInformation = 4;
constexpr int kNumberAttempts = 4;
constexpr int kTimeOut = 10; // ms.

}  // namespace

absl::Status NogginGpr::BootNogginGprDevice() {
  serial_->write(kOnCmd);
  ros::Duration(3).sleep();
  std::string buff = serial_->read(serial_->available());
  ROS_INFO_STREAM(buff << std::endl);
  if (buff.find(kOnStateIndicator) != std::string::npos || 
      absl::StrContains(buff, kOnStateIndicator)) {
    return absl::OkStatus();
  }
  serial_->write(kReportCmd);
  ros::Duration(3).sleep();
  buff = serial_->read(serial_->available());
  ROS_INFO_STREAM(buff << std::endl);
  if (buff.find(kOnStateIndicator) != std::string::npos || 
      absl::StrContains(buff, kOnStateIndicator)) {
    return absl::OkStatus();
  }
  return absl::InternalError("Unable to boot Noggin GPR device.");
}

NogginGpr::NogginGpr() : ros_radar_(NogginGprRos()),
                         trace_vector_size_(
                           ros_radar_.noggin_config_.points 
                           + kAdditionalInformation) {
  ROS_INFO_STREAM("Opening port: " << kSerialPortPath);
  // serial_ = new serial::Serial(kSerialPortPath, 
  //                          ros_radar_.noggin_config_.baud_rate,
  //                          serial::Timeout::simpleTimeout(kTimeOut));
  
  serial_ = new serial::Serial();
  for (int i=0; i < kNumberAttempts; i++) {
    try {
      ROS_INFO_STREAM("Opening radar USB port: " << kSerialPortPath);
      serial_->setPort(kSerialPortPath);
      serial_->setBaudrate(ros_radar_.noggin_config_.baud_rate);
      serial::Timeout to = serial::Timeout::simpleTimeout(kTimeOut);
      serial_->setTimeout(to);
      serial_->open();
      if (serial_->isOpen()) break;
    } 
    catch(serial::IOException& e) {
      ROS_INFO_STREAM("Unable to open radar USB port: " << kSerialPortPath);
    }
    ros::Duration(1.5).sleep();
  }
  CHECK(serial_->isOpen()) << "Unable to open radar USB port" << kSerialPortPath;
  ROS_INFO_STREAM("Opened radar USB port: " << kSerialPortPath);

  if (!ros_radar_.noggin_config_.bypass_device_startup) {
    ROS_INFO_STREAM("Beginning to autobaud for 10s.");
    for (int i=0; i < 10; i++) {
      serial_->write("\r");
      ros::Duration(1).sleep();
    }
    ROS_INFO_STREAM("Autobauding complete.");

    absl::Status status;
    for (int i=0; i < kNumberAttempts; i++) {
      status = this->BootNogginGprDevice();
      if (status.ok()) break;
    }
    CHECK(status.ok()) << "Unable to boot Noggin GPR Device "
                      << ros_radar_.noggin_config_.device_id;
    ROS_INFO_STREAM("Booted Noggin GPR Device " 
                    << ros_radar_.noggin_config_.device_id);

    ROS_INFO_STREAM("Entering blocking time for 10s.");
    ros::Time release = ros::Time::now() + ros::Duration(10);
    while (ros::Time::now() < release) {
      if (serial_->available()) {
        serial_->read(serial_->available());
      }
      ros::Duration(0.5).sleep();
    }
  }
  ROS_INFO_STREAM("Noggin GPR device intialized.");
}

int NogginGpr::GetSamplingFrequency() {
  return ros_radar_.noggin_config_.sampling_frequency;
}

void NogginGpr::RequestTrace() {
  mu_.Lock();
  ready_to_sample_ = false;
  serial_->flush();
  ros::Time time = ros::Time::now();
  serial_->write(kRequestTraceCmd);
  ros::Time trace_acquisition_time = ros::Time::now();
  uint8_t result_buffer[trace_vector_size_];
  
  bool acquired_data = false;
  std::string result;
  while (!acquired_data) {
    if (serial_->available() && serial_->available() > 2) {
      ROS_INFO_STREAM(serial_->available());
      ROS_INFO_STREAM(ros::Time::now() - time);
      result = serial_->read(serial_->available());
      acquired_data = true;
    } if (ros::Time::now() - time > ros::Duration(2)) {
      ready_to_sample_ = true;
      mu_.Unlock();
      return;
    }
  }
  ready_to_sample_ = true;
  size_t starting_position = result.find(kTracePrefix);
  std::string raw_trace;
  if (starting_position != std::string::npos &&
      starting_position  + strlen(kTracePrefix)
      + 2 * ros_radar_.noggin_config_.points < result.length()) {
    raw_trace = result.substr(starting_position + strlen(kTracePrefix),
                              2 * ros_radar_.noggin_config_.points);
  }
  
  ROS_INFO_STREAM(raw_trace);

  // ROS_INFO_STREAM(serial_->read(serial_->available()));
  // int bytes_back = serial_->read(result_buffer, trace_vector_size_);
  // int16_t* signed_trace = reinterpret_cast<int16_t*>(result_buffer);
  // std::vector<int16_t> out;
  // out.assign(signed_trace, signed_trace + (trace_vector_size_/2));
  // ros_radar_.PublishTrace(trace_acquisition_time, out);
  mu_.Unlock();
}

}  // namespace lgpr::noggin