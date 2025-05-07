#pragma once

#include <linux/can.h>
#include <string>

#include "common/motor_states.h"

namespace humanoid {
namespace can_driver {

class CanReaderBase {
 public:
  CanReaderBase() = default;

  virtual ~CanReaderBase() = default;

  virtual bool Init(const std::string& name, canid_t id);

  virtual bool ReadCanFrame(const can_frame& frame) = 0;

  canid_t id() { return id_; }

  MotorState motor_state() { return motor_info_; }

  std::string name() { return name_; }

 protected:
  MotorState motor_info_;

  std::string name_;

  canid_t id_;
};

}  // namespace can_driver
}  // namespace humanoid