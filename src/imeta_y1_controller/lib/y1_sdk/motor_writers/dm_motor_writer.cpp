#include "motor_writers/dm_motor_writer.h"

#include <cmath>

#include "common/log.h"
#include "common/motor_parameters.h"

namespace imeta {
namespace controller {

bool DmMotorWriter::Init(const MotorInfo& motor_info) {
  if (!MotorWriterBase::Init(motor_info)) {
    AERROR << "Failed to CanWriterBase::Init";
    return false;
  }

  AINFO << name() << " min position limit: " << motor_info_.position_min
        << " , max position limit: " << motor_info_.position_max;

  return true;
}

bool DmMotorWriter::WriteCanFrame(can_frame& frame,
                                  const MitControlMode& control_command) {
  // limit
  double position_limit = std::max(
      position_min_, std::min(control_command.position, position_max_));

  uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
  pos_tmp = float_to_uint(position_limit, DM_P_MIN, DM_P_MAX, 16);
  vel_tmp = float_to_uint(control_command.velocity, DM_V_MIN, DM_V_MAX, 12);
  kp_tmp = float_to_uint(control_command.kp, DM_KP_MIN, DM_KP_MAX, 12);
  kd_tmp = float_to_uint(control_command.kd, DM_KD_MIN, DM_KD_MAX, 12);
  tor_tmp = float_to_uint(control_command.torque, DM_T_MIN, DM_T_MAX, 12);

  frame.can_id = id_;
  frame.can_dlc = 8;
  frame.data[0] = (pos_tmp >> 8);
  frame.data[1] = pos_tmp;
  frame.data[2] = (vel_tmp >> 4);
  frame.data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
  frame.data[4] = kp_tmp;
  frame.data[5] = (kd_tmp >> 4);
  frame.data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
  frame.data[7] = tor_tmp;

  return true;
}

int DmMotorWriter::float_to_uint(float x, float x_min, float x_max,
                                 int bits) const {
  /// Converts a float to an unsigned int, given range and number of bits
  float span = x_max - x_min;
  float offset = x_min;

  return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

void DmMotorWriter::Enable(can_frame& frame) {
  frame.can_id = id_;
  frame.can_dlc = 8;
  frame.data[0] = 0xFF;
  frame.data[1] = 0xFF;
  frame.data[2] = 0xFF;
  frame.data[3] = 0xFF;
  frame.data[4] = 0xFF;
  frame.data[5] = 0xFF;
  frame.data[6] = 0xFF;
  frame.data[7] = 0xFC;
}

void DmMotorWriter::Disable(can_frame& frame) {
  frame.can_id = id_;
  frame.can_dlc = 8;
  frame.data[0] = 0xFF;
  frame.data[1] = 0xFF;
  frame.data[2] = 0xFF;
  frame.data[3] = 0xFF;
  frame.data[4] = 0xFF;
  frame.data[5] = 0xFF;
  frame.data[6] = 0xFF;
  frame.data[7] = 0xFD;
}

}  // namespace controller
}  // namespace imeta