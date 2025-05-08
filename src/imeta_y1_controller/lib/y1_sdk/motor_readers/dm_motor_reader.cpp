#include "motor_readers/dm_motor_reader.h"

#include "common/log.h"

namespace imeta {
namespace controller {

bool DmMotorReader::Init(const MotorInfo& motor_info) {
  if (!MotorReaderBase::Init(motor_info)) {
    AERROR << "Failed to CanReaderBase::Init";
    return false;
  }

  return true;
}

bool DmMotorReader::ReadCanFrame(const can_frame& frame) {
  if (frame.can_id != motor_info_.motor_read_info.id) {
    AERROR << "can id not matched";
    return false;
  }

  auto frame_data = frame.data;
  uint16_t p_int, v_int, t_int;
  p_int = (frame_data[1] << 8) | frame_data[2];
  v_int = (frame_data[3] << 4) | (frame_data[4] >> 4);
  t_int = ((frame_data[4] & 0xF) << 8) | frame_data[5];

  // convert
  double pmax = motor_info_.pmax;
  double vmax = motor_info_.vmax;
  double tmax = motor_info_.tmax;
  motor_state_.position = uint_to_float(p_int, -pmax, pmax, 16);
  motor_state_.velocity = uint_to_float(v_int, -vmax, vmax, 12);
  motor_state_.torque = uint_to_float(t_int, -tmax, tmax, 12);

  return true;
}

float DmMotorReader::uint_to_float(int x_int, float x_min, float x_max,
                                   int bits) {
  // converts unsigned int to float, given range and number of bits
  float span = x_max - x_min;
  float offset = x_min;

  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

}  // namespace controller
}  // namespace imeta