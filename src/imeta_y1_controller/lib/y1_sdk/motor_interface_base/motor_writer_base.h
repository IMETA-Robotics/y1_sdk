#pragma once

#include <linux/can.h>

#include <string>

// #include "common/motor_control_command.h"
#include "config/motor_config.h"

namespace imeta {
namespace controller {

struct MitControlMode {
  double position;
  double velocity;
  double kp;
  double kd;
  double torque;
};

class MotorWriterBase {
 public:
  MotorWriterBase() = default;

  virtual ~MotorWriterBase() = default;

  virtual bool Init(const MotorInfo& motor_info);

  /**
   * * @brief
   */
  virtual void MitControl(can_frame& frame,
                          const MitControlMode& control_command) = 0;

  /**
   * * @brief return enable motor can frame
   */
  virtual void Enable(can_frame& frame) = 0;

  /**
   * * @brief return disable motor can frame
   */
  virtual void Disable(can_frame& frame) = 0;

  /**
   * * @brief return set zero joint position can frame
   */
  virtual void SetZeroPosition(can_frame& frame) = 0;

  std::string name() { return motor_info_.joint_name; }

 protected:
  MotorInfo motor_info_;
};

}  // namespace controller
}  // namespace imeta