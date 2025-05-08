#pragma once

#include <linux/can.h>

#include <string>

#include "common/motor_state.h"
#include "config/motor_config.h"

namespace imeta {
namespace controller {

class MotorReaderBase {
 public:
  MotorReaderBase() = default;

  virtual ~MotorReaderBase() = default;

  virtual bool Init(const MotorInfo& motor_info);

  virtual bool ReadCanFrame(const can_frame& frame) = 0;

  canid_t id() { return motor_info_.motor_read_info.id; }

  MotorState motor_state() { return motor_state_; }

  std::string name() { return motor_info_.joint_name; }

 protected:
  MotorState motor_state_;

  MotorInfo motor_info_;
};

}  // namespace controller
}  // namespace imeta