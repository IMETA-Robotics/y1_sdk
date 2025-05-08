#pragma once

#include "motor_interface_base/motor_reader_base.h"

namespace imeta {
namespace controller {

class DmMotorReader : public MotorReaderBase {
 public:
  DmMotorReader() = default;

  ~DmMotorReader() = default;

  bool Init(const MotorInfo& motor_info) override;

  bool ReadCanFrame(const can_frame& frame) override;

 private:
  float uint_to_float(int x_int, float x_min, float x_max, int bits);
};

}  // namespace controller
}  // namespace imeta