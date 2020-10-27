#include "noggin_gpr_node/noggin_gpr.h"

#include <glog/logging.h>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <cmath>
#include <serial/serial.h>
#include <absl/strings/match.h>
#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "absl/base/thread_annotations.h"
#include "absl/strings/str_format.h"
#include "ros/time.h"


namespace lgpr::noggin {

namespace {

constexpr char kRequestTraceCmd[] = "T\r";
constexpr char kOnCmd[] = "XON\r";
constexpr char kReportCmd[] = "R\r";
constexpr char kOnStateIndicator[] = "TPU";
constexpr char kTracePrefix[] = "T01";
// TODO(abaikovitz) Clean up serial comms.
constexpr int kAdditionalInformation = 6;
constexpr int kInitialBytes = 3;
constexpr int kNumberAttempts = 4;
constexpr int kTimeOut = 100; // ms.

std::vector<int16_t> FormatTrace(const std::string& raw_trace,
                                 const int& points) {
  std::vector<int16_t> out(points);
  int16_t trace_element;
  for (int i = 0; i < 2 * points; i++) {
    if(((kInitialBytes + i) % 2) == 1) {
      trace_element = 0;
      trace_element = trace_element | static_cast<uint8_t>(raw_trace.at(kInitialBytes + i)) << 8;
    } else {
      out[i / 2] = trace_element | static_cast<uint8_t>(raw_trace.at(kInitialBytes + i));
    }
  } 
  return out;
}

void analyze_trace(uint8_t* input, int num_points, std::vector<int16_t> &trace) {
    int start = 3;
    int16_t trace_unit{0};
    for (int i = start; i < start + num_points*2;i++) {
        if((i % 2) == 1) {
            trace_unit = 0;
            // std::bitset<16> t1(input[i]);
            // std::cout << "number input " << t1 << "\n";
            trace_unit = trace_unit | (input[i] << 8);
            // ROS_INFO_STREAM("INT Received" + std::to_string(trace_unit));
            // std::bitset<16> t2(trace_unit);
            // std::cout << "number output " << t2 << "\n";
        }

        else {
            trace_unit = trace_unit | input[i];
            // ROS_INFO_STREAM("INT Received" + std::to_string(trace_unit));
            trace[(i-start) / 2] = trace_unit;
        }
    }
}

void InitializeSerialPort(serial::Serial* serial, const NogginGprConfig& config) {
  for (int i=0; i < kNumberAttempts; i++) {
    try {
      ROS_INFO_STREAM("Opening radar USB port: " << config.serial_port_id);
      serial->setPort(config.serial_port_id);
      serial->setBaudrate(config.baud_rate);
      serial::Timeout to = serial::Timeout::simpleTimeout(kTimeOut);
      serial->setTimeout(to);
      serial->open();
      if (serial->isOpen()) break;
    } 
    catch(serial::IOException& e) {
      ROS_INFO_STREAM("Unable to open radar USB port: " << config.serial_port_id);
    }
    ros::Duration(1.5).sleep();
  }
  CHECK(serial->isOpen()) << "Unable to open radar USB port" << config.serial_port_id;


}

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
                           2*ros_radar_.noggin_config_.points 
                           + kAdditionalInformation) {
  ROS_INFO_STREAM("Opening port: " << ros_radar_.noggin_config_.serial_port_id);

  serial_ = new serial::Serial();
  InitializeSerialPort(serial_, ros_radar_.noggin_config_);
  // Reset serial buffer.
  serial_->flush();
  ROS_INFO_STREAM("Opened radar USB port: " << ros_radar_.noggin_config_.serial_port_id);

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

    // Set number of points queried from Noggin.
    std::string points_cmd = 
      absl::StrFormat("%d P\r", ros_radar_.noggin_config_.points);
    for (int i=0; i < kNumberAttempts; i++) {
      serial_->write(points_cmd.c_str());
    }
    ROS_INFO_STREAM("Wrote \"" << points_cmd << "\" to Noggin device.");

    ros::Duration(4).sleep();

    // Set starting offset from Noggin.
    
    std::string offset_start_cmd;
    int offset = std::abs(ros_radar_.noggin_config_.point_offset);
    ros_radar_.noggin_config_.point_offset < 0 ? 
      offset_start_cmd = absl::StrFormat("%d-\r", offset) : 
      offset_start_cmd = absl::StrFormat("%d+\r", offset);

    for (int i=0; i < kNumberAttempts; i++) {
      serial_->write(offset_start_cmd.c_str());
    }
    ROS_INFO_STREAM("Wrote \"" << offset_start_cmd << "\" to Noggin device.");
    ros::Duration(4).sleep();
  }
  ROS_INFO_STREAM("Noggin GPR device intialized.");
}

int NogginGpr::GetSamplingFrequency() {
  return ros_radar_.noggin_config_.sampling_frequency;
}

bool NogginGpr::RequestTrace() {
  mu_.Lock();
  serial_->flush();
  ros::Time time = ros::Time::now();
  serial_->write(kRequestTraceCmd);
  ros::Time trace_acquisition_time = ros::Time::now();
  
  std::string result = serial_->read(trace_vector_size_);
  // ROS_INFO_STREAM("Result length " << result.length());

  mu_.Unlock();
  size_t starting_position = result.find(kTracePrefix);
  std::string raw_trace;
  if (result.length() == trace_vector_size_) {
    // raw_trace = result.substr(starting_position + strlen(kTracePrefix), 
    //                           2 * ros_radar_.noggin_config_.points);
    // std::vector<int16_t> out = FormatTrace(raw_trace, 
    //                                        ros_radar_.noggin_config_.points);
    std::vector<int16_t> out = FormatTrace(result, ros_radar_.noggin_config_.points);
    ros_radar_.PublishTrace(trace_acquisition_time, out);
    return true;
  }
  return false;
}


// void NogginGpr::RequestTrace() {
//     mu_.Lock();
//     int points = 200;
//     unsigned int additional_factors{8};
//     unsigned int size{points*2 + additional_factors};

//     serial_->write("T\r");

//     ros::Time sensor_acquisition_time = ros::Time::now();

//     ROS_INFO_STREAM("Reading trace from serial");
//     uint8_t result[size]{};

//     int bytes_back = serial_->read(result,size);
//     ROS_INFO_STREAM(bytes_back);
//     if (bytes_back == size) {
//       // ROS_INFO_STREAM("Analyzing Trace");
//         std::vector<int16_t> trace(points,0);
//         analyze_trace(result,points,trace);
//         ros_radar_.PublishTrace(sensor_acquisition_time, trace);
//     }
//     mu_.Unlock();
// }



// void NogginGpr::RequestTrace() {
  // mu_.Lock();
  // serial_->write("T\r");
  // ros::Time trace_acquisition_time = ros::Time::now();
  // std::string result;
  // while (result.length() < 405) {
  //   if (serial_->available())
  //     result = result + serial_->read(serial_->available());
  //     if (result.length() >= 408) {
  //       std::vector<int16_t> trace = FormatTrace(result, 200);
  //       ros_radar_.PublishTrace(trace_acquisition_time, trace);
  //   }
  // }

  // serial_->write(kRequestTraceCmd);
  // ros::Time trace_acquisition_time = ros::Time::now();
  // uint8_t result_buffer[408]{0};

  // bool acquired_data = false;
  // std::string result;
  // while (!acquired_data) {
    
  //   if (serial_->available()) {
  //     int bytes_back = serial_->read(result_buffer, 408);
  //     if (bytes_back == 408) {
  //       std::vector<int16_t> out(201, 0);
  //       analyze_trace(result_buffer, 201, out);
  //       ros_radar_.PublishTrace(trace_acquisition_time, out);
  //     }
  //     // ROS_INFO_STREAM("received");
  //     // result = serial_->read(serial_->available());
  //     acquired_data = true;
  //     ROS_INFO_STREAM(strlen(kTracePrefix) << " " << result.length() << " " << result);
  //   } if (ros::Time::now() - time > ros::Duration(2)) {
  //     mu_.Unlock();
  //     return;
  //   }
  // }

  // int bytes_back = serial_->read(result_buffer, 408);
  // ROS_INFO_STREAM(bytes_back);
  // if (bytes_back == 408) {
  //   std::vector<int16_t> out(201, 0);
  //   analyze_trace(result_buffer, 201, out);
  //   ros_radar_.PublishTrace(trace_acquisition_time, out);
  // }
  // if (bytes_back == 408) {
  //   out = FormatTrace(result_buffer, ros_radar_.noggin_config_.points);
  //   ros_radar_.PublishTrace(trace_acquisition_time, out);
  // }

  // ready_to_sample_ = true;
  // if (result.length() > 0) {
  //   size_t starting_position = result.find(kTracePrefix);
  //   int16_t *raw_trace;
  //   std::string raw_trace_str;
  //   std::vector<int16_t> out;
  //   if (starting_position != std::string::npos && 
  //       starting_position + strlen(kTracePrefix)
  //       + 2 * ros_radar_.noggin_config_.points < result.length()) {
  //     raw_trace_str = result.substr(starting_position + strlen(kTracePrefix),
  //                               2 * ros_radar_.noggin_config_.points);
  //     out = FormatTrace(raw_trace_str, ros_radar_.noggin_config_.points);
  //     ros_radar_.PublishTrace(trace_acquisition_time, out);
  //   }
  // }


  // ROS_INFO_STREAM(serial_->read(serial_->available()));
  // int bytes_back = serial_->read(result_buffer, trace_vector_size_);
  // int16_t* signed_trace = reinterpret_cast<int16_t*>(result_buffer);
  // std::vector<int16_t> out;
  // out.assign(signed_trace, signed_trace + (trace_vector_size_/2));
  // ros_radar_.PublishTrace(trace_acquisition_time, out);

//   mu_.Unlock();
// }

}  // namespace lgpr::noggin