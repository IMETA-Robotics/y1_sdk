#include "can_readers/dm_motor_reader.h"

#include "common/log.h"
#include "common/motor_parameters.h"

namespace humanoid {
namespace can_driver {

bool DmMotorReader::Init(const std::string& name, canid_t id) {
  if (!CanReaderBase::Init(name, id)) {
    AERROR << "Failed to CanReaderBase::Init";
    return false;
  }

  return true;
}

bool DmMotorReader::ReadCanFrame(const can_frame& frame) {
  if (frame.can_id != id_) {
    AERROR << "can id not matched";
    return false;
  }

  auto frame_data = frame.data;
  uint16_t p_int, v_int, t_int;
  p_int = (frame_data[1] << 8) | frame_data[2];
  v_int = (frame_data[3] << 4) | (frame_data[4] >> 4);
  t_int = ((frame_data[4] & 0xF) << 8) | frame_data[5];

  // convert
  motor_info_.position = uint_to_float(p_int, DM_P_MIN, DM_P_MAX, 16);
  motor_info_.velocity = uint_to_float(v_int, DM_V_MIN, DM_V_MAX, 12);
  motor_info_.torque = uint_to_float(t_int, DM_T_MIN, DM_T_MAX, 12);

  return true;
}

float DmMotorReader::uint_to_float(int x_int, float x_min, float x_max,
                                   int bits) {
  // converts unsigned int to float, given range and number of bits
  float span = x_max - x_min;
  float offset = x_min;

  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

}  // namespace can_driver
}  // namespace humanoid