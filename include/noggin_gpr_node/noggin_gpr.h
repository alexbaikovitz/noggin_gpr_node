#ifndef NOGGIN_GPR_H
#define NOGGIN_GPR_H

#include "noggin_gpr_node/noggin_gpr_ros.h"

#include <serial/serial.h>
#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "absl/base/thread_annotations.h"
#include <memory>

namespace lgpr::noggin {

class NogginGpr {
  public:
    NogginGpr();
    ~NogginGpr() = default;

    bool RequestTrace() LOCKS_EXCLUDED(mu_);
    int GetSamplingFrequency();

    NogginGpr(const NogginGpr&);
    NogginGpr& operator=(const NogginGpr&);

    bool ready_to_sample_ GUARDED_BY(mu_);

  private:
    NogginGprRos ros_radar_;
    const int trace_vector_size_;
    serial::Serial* serial_ GUARDED_BY(mu_);
    absl::Mutex mu_;

    absl::Status BootNogginGprDevice();
    absl::Status OpenSerialPort();
};

}  // namespace lgpr::noggin


#endif
