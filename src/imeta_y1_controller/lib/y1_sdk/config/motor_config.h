#pragma once

#include <linux/can.h>

#include <string>
#include <vector>

namespace imeta {
namespace controller {

struct MotorReadInfo {
  canid_t id;
  std::string class_type;
};

struct MotorWriteInfo {
  canid_t id;
  std::string class_type;
};

struct MotorInfo {
  // correspond to (J1, J2, J3, J4, J5, J6, Gripper)
  std::string joint_name;
  MotorReadInfo motor_read_info;
  MotorWriteInfo motor_write_info;
  // 电机控制幅度
  double pmax;
  double vmax;
  double tmax;
  // min and max joint position limit（unit：degree）
  double position_min;
  double position_max;
};

// 外部声明配置变量
extern const std::vector<MotorInfo> kMotorInfos;

}  // namespace controller
}  // namespace imeta