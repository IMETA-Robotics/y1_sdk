#pragma once

#include <linux/can.h>
#include <string>

#include "common/motor_control_command.h"

#include "proto/can_driver_config.pb.h"

namespace humanoid {
namespace can_driver {

class CanWriterBase {
 public:
  CanWriterBase() = default;

  explicit CanWriterBase(const DeviceInfo& device_info);

  virtual ~CanWriterBase() = default;

  virtual bool Init(const std::string& name, canid_t id, int socket);

  virtual bool WriteCanFrame(can_frame& frame,
                             const MitControlMode& control_command) = 0;

  virtual void Enable(can_frame& frame) = 0;

  virtual void Disable(can_frame& frame) = 0;

  virtual int socket() { return socket_; }

  std::string name() { return name_; }

 protected:
  std::string name_;

  canid_t id_;

  int socket_;

  double position_min_;

  double position_max_;

  DeviceInfo device_info_;
};

}  // namespace can_driver
}  // namespace humanoid