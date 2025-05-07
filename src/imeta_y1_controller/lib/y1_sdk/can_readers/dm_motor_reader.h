#pragma once

#include "can_interface_base/can_reader_base.h"

namespace humanoid {
namespace can_driver {

class DmMotorReader : public CanReaderBase {
 public:
  DmMotorReader() = default;

  ~DmMotorReader() = default;

  bool Init(const std::string& name, canid_t id) override;

  bool ReadCanFrame(const can_frame& frame) override;

 private:
  float uint_to_float(int x_int, float x_min, float x_max, int bits);
};

}  // namespace can_driver
}  // namespace humanoid