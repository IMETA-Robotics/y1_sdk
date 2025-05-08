#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "motor_interface_base/motor_reader_base.h"
#include "motor_interface_base/motor_writer_base.h"
// #include "common/motor_control_command.h"
// #include "common/motor_states.h"

namespace imeta {
namespace controller {

class CanManager {
 public:
  CanManager() = delete;

  explicit CanManager(const std::string& can_id, const std::string& urdf_path, int arm_end_type);

  ~CanManager();

  bool Init();

  // void RunOnce(const ControlCommandVector& control_command,
  //              MotorStateVector& motor_states_info);

 private:
  bool OpenCanDevice(const std::string& can_device_id, int& socket);

  void GenerateThread();

  bool WriteCanFrame(const can_frame& frame, int socket) const;

 private:
  // socket can
  std::atomic<int> socket_;

  // std::unordered_map<std::string, int> socket_can_map_;

  // reader threads
  std::thread reader_thread_;
  std::mutex reader_mutux_;
  std::atomic<bool> stop_flag_ = false;

  // readers and writers
  std::vector<std::shared_ptr<MotorReaderBase>> motor_readers_;
  std::vector<std::unique_ptr<MotorWriterBase>> motor_writers_;

  // config
  std::string can_id_;
  int arm_end_type_;
  // debug
  int count_success_ = 0;
  int count_fail_ = 0;
};

}  // namespace can_driver
}  // namespace humanoid