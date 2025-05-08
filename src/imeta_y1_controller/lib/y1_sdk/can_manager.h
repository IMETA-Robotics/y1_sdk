#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "motor_interface_base/motor_reader_base.h"
#include "motor_interface_base/motor_writer_base.h"
// #include "common/motor_states.h"

namespace imeta {
namespace controller {

class CanManager {
 public:
  CanManager() = delete;

  explicit CanManager(const std::string& can_id, const std::string& urdf_path,
                      int arm_end_type);

  ~CanManager();

  bool Init();

  std::vector<double> GetJointPosition();

  std::vector<double> GetJointVelocity();

  std::vector<double> GetJointEffort();

  std::array<double, 6> GetArmEndPose();

  void SetArmControlMode(int mode);

  void SetArmJointPosition(const std::array<double, 6>& arm_joint_position);

  void SetArmEndPose(const std::array<double, 6>& arm_end_pose);

  void SetGripperJointPosition(double gripper_joint_position);

 private:
  bool OpenCanDevice(const std::string& can_id, int& socket);

  void GenerateReaderThread();

  void GenerateControlThread();

  bool WriteCanFrame(const can_frame& frame, int socket) const;

 private:
  // socket can
  std::atomic<int> socket_;

  // thread
  std::thread control_thread_;
  std::thread reader_thread_;
  std::mutex reader_mutux_;
  std::atomic<bool> stop_flag_ = false;

  // readers and writers
  std::vector<std::shared_ptr<MotorReaderBase>> motor_readers_;
  std::vector<std::unique_ptr<MotorWriterBase>> motor_writers_;

  // config
  std::string can_id_;
  int arm_end_type_;

  // data
  MotorStateVector motor_states_;
  int arm_control_mode_ = 1;  // default: joint position control
  std::array<double, 6> target_joint_position_;
  double target_gripper_joint_position_;
  // std::vector<double> target_end_pose_;

  // debug
  int count_success_ = 0;
  int count_fail_ = 0;
};

}  // namespace controller
}  // namespace imeta