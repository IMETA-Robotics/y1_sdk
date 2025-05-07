#pragma once

#include <string>
#include <vector>

namespace imeta {
namespace controller {

struct MotorReadInfo {
  unsigned int id;
  std::string type;
};

struct MotorWriteInfo {
  unsigned int id;
  std::string type;
};

struct MotorInfo {
  std::string joint_name;            // correspond to the joint name(J1, J2, J3, J4, J5, J6, Gripper)
  // unsigned int read_id;           // read motor data
  // unsigned int write_id;          // write data to control motor
  MotorReadInfo motor_read_info;
  MotorWriteInfo motor_write_info;
  double position_min;               // min joint position limit（unit：degree）
  double position_max;               // max joint position limit（unit：degree）
};

// 定义单个CAN设备配置
struct CanConfig {
  std::string can_id;
  std::vector<MotorInfo> motor_devices;
};

// 外部声明配置变量
extern const std::vector<CanConfig> CanConfigs;

}
}