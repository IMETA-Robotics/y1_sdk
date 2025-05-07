#include "can_interface_base/can_writer_base.h"

namespace humanoid {
namespace can_driver {

CanWriterBase::CanWriterBase(const DeviceInfo& device_info)
    : device_info_(device_info) {}

bool CanWriterBase::Init(const std::string& name, canid_t id, int socket) {
  name_ = name;
  id_ = id;
  socket_ = socket;

  return true;
}

}  // namespace can_driver
}  // namespace humanoid