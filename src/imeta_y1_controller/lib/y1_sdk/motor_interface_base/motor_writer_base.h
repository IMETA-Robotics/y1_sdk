#pragma once

#include <linux/can.h>

#include <string>

#include "config/motor_config.h"
#include "common/motor_control_command.h"

namespace imeta {
namespace controller {

class MotorWriterBase {
 public:
  MotorWriterBase() = default;

  virtual ~MotorWriterBase() = default;

  virtual bool Init(const MotorInfo& motor_info);

  virtual bool WriteCanFrame(can_frame& frame,
                             const MitControlMode& control_command) = 0;

  virtual void Enable(can_frame& frame) = 0;

  virtual void Disable(can_frame& frame) = 0;

  // virtual int socket() { return socket_; }

  std::string name() { return motor_info_.joint_name; }

 protected:

  MotorInfo motor_info_;
};

}  // namespace controller
}  // namespace imeta