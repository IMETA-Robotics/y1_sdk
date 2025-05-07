#pragma once

#include "can_interface_base/can_writer_base.h"

namespace humanoid {
namespace can_driver {

class DmMotorWriter : public CanWriterBase {
 public:
  DmMotorWriter() = default;

  explicit DmMotorWriter(const DeviceInfo& device_info);

  ~DmMotorWriter() = default;

  bool Init(const std::string& name, canid_t id, int socket) override;

  bool WriteCanFrame(can_frame& frame,
                     const MitControlMode& control_command) override;

  void Enable(can_frame& frame) override;

  void Disable(can_frame& frame) override;

 private:
  int float_to_uint(float x, float x_min, float x_max, int bits) const;
};

}  // namespace can_driver
}  // namespace humanoid