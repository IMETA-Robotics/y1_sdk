#include "motor_interface_base/motor_reader_base.h"

namespace imeta {
namespace controller {

bool MotorReaderBase::Init(const MotorInfo& motor_info) {
  motor_info_ = motor_info;

  return true;
}

}  // namespace controller
}  // namespace imeta