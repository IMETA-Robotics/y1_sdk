#pragma once

#include "motor_interface_base/motor_writer_base.h"

namespace imeta {
namespace controller {

class DmMotorWriter : public MotorWriterBase {
 public:
  DmMotorWriter() = default;

  ~DmMotorWriter() = default;

  bool Init(const MotorInfo& motor_info) override;

  bool WriteCanFrame(can_frame& frame,
                     const MitControlMode& control_command) override;

  void Enable(can_frame& frame) override;

  void Disable(can_frame& frame) override;

 private:
  int float_to_uint(float x, float x_min, float x_max, int bits) const;
};

}  // namespace controller
}  // namespace imeta