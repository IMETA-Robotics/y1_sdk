#include "can_interface_base/can_reader_base.h"

namespace humanoid {
namespace can_driver {

bool CanReaderBase::Init(const std::string& name, canid_t id) {
  name_ = name;
  id_ = id;

  return true;
}

}  // namespace can_driver
}  // namespace humanoid