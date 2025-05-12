#include "can_manager.h"

#include <linux/can.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <chrono>
#include <cstring>
#include <thread>

#include "common/log.h"
#include "config/motor_config.h"
#include "motor_interface_base/motor_writer_base.h"
#include "motor_readers/dm_motor_reader.h"
#include "motor_writers/dm_motor_writer.h"

namespace imeta {
namespace controller {

CanManager::CanManager(const std::string& can_id, const std::string& urdf_path,
                       int arm_end_type)
    : can_id_(can_id), arm_end_type_(arm_end_type) {}

CanManager::~CanManager() {
  stop_flag_.store(true);

  // Disable motor
  can_frame frame;
  for (int i = 0; i < motor_writers_.size(); i++) {
    motor_writers_.at(i)->Disable(frame);
    if (!WriteCanFrame(frame, socket_.load())) {
      AERROR << "Failed to disable motor: " << motor_writers_.at(i)->name();
    }
  }

  // wait for reader thread finish
  if (reader_thread_.joinable()) {
    AINFO << std::this_thread::get_id() << " reader_thread join";
    reader_thread_.join();
  }

  if (control_thread_.joinable()) {
    AINFO << std::this_thread::get_id() << " control_thread join";
    control_thread_.join();
  }
}

bool CanManager::Init() {
  // open can device
  int socket;
  if (!OpenCanDevice(can_id_, socket)) {
    AERROR << "Failed to open can device:" << can_id_;
    return false;
  }
  socket_.store(socket);

  // if load end motor
  bool load_end_motor;
  if (arm_end_type_ == 0) {
    load_end_motor = false;
  } else if (arm_end_type_ == 1 || arm_end_type_ == 2) {
    load_end_motor = true;
  } else {
    AERROR << "arm_end_type is error: " << arm_end_type_;
    return false;
  }

  if (kMotorInfos.size() != 7) {
    AERROR << "motor config size != 7";
    return false;
  }

  int motor_size = load_end_motor ? kMotorInfos.size() : kMotorInfos.size() - 1;
  for (int i = 0; i < motor_size; i++) {
    // reader
    auto motor_info = kMotorInfos.at(i);
    if (motor_info.motor_read_info.class_type == "DmMotorReader") {
      std::shared_ptr<MotorReaderBase> reader =
          std::make_shared<DmMotorReader>();
      reader->Init(motor_info);

      motor_readers_.emplace_back(reader);
    } else {
      AERROR << "Failed create " << motor_info.motor_read_info.class_type;
      return false;
    }

    // writer
    if (motor_info.motor_write_info.class_type == "DmMotorWriter") {
      std::unique_ptr<MotorWriterBase> writer =
          std::make_unique<DmMotorWriter>();
      writer->Init(motor_info);

      motor_writers_.emplace_back(std::move(writer));
    } else {
      AERROR << "Failed create " << motor_info.motor_write_info.class_type;
      return false;
    }
  }

  // generate one thread for can read data
  reader_thread_ = std::thread(&CanManager::GenerateReaderThread, this);
  // generate one thread for control motor and save motor state
  control_thread_ = std::thread(&CanManager::GenerateControlThread, this);

  // motor enable
  can_frame frame;
  for (int i = 0; i < motor_writers_.size(); i++) {
    motor_writers_.at(i)->Enable(frame);
    if (!WriteCanFrame(frame, socket_.load())) {
      AERROR << "Failed to enable device: " << motor_writers_.at(i)->name();
      return false;
    }
  }

  // The position at the time of arm power-on is taken as initial joint
  // position.
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  std::vector<double> joint_position = GetJointPosition();
  if (joint_position.size() < 6) {
    AERROR << "ERROR: arm joint position size is " << joint_position.size();
    return false;
  }

  for (int i = 0; i < 6; ++i) {
    target_joint_position_.at(i) = joint_position.at(i);
  }

  if (load_end_motor) {
    if (joint_position.size() == 7) {
      target_gripper_joint_position_ = joint_position.back();
    } else {
      AERROR << "ERROR: need load end motor, but joint position size is "
             << joint_position.size();
      return false;
    }
  }

  return true;
}

std::vector<double> CanManager::GetJointPosition() {
  std::vector<double> joint_position;
  {
    std::lock_guard<std::mutex> lock(reader_mutux_);
    for (int i = 0; i < motor_readers_.size(); i++) {
      joint_position.emplace_back(motor_readers_.at(i)->motor_state().position);
    }
  }

  return joint_position;
}

std::vector<double> CanManager::GetJointVelocity() {
  std::vector<double> joint_velocity;
  {
    std::lock_guard<std::mutex> lock(reader_mutux_);
    for (int i = 0; i < motor_readers_.size(); i++) {
      joint_velocity.emplace_back(motor_readers_.at(i)->motor_state().velocity);
    }
  }

  return joint_velocity;
}

std::vector<double> CanManager::GetJointEffort() {
  std::vector<double> joint_effort;
  {
    std::lock_guard<std::mutex> lock(reader_mutux_);
    for (int i = 0; i < motor_readers_.size(); i++) {
      joint_effort.emplace_back(motor_readers_.at(i)->motor_state().torque);
    }
  }

  return joint_effort;
}

std::array<double, 6> CanManager::GetArmEndPose() {
  std::array<double, 6> arm_joint_position;
  {
    std::lock_guard<std::mutex> lock(reader_mutux_);
    for (int i = 0; i < 6; i++) {
      arm_joint_position.at(i) = (motor_readers_.at(i)->motor_state().position);
    }
  }

  std::array<double, 6> end_pose;
  // TODO: arm kinematics compute end pose

  return end_pose;
}

void CanManager::SetArmControlMode(int mode) {
  // ControlMode
  if (mode == 0) {
    // GO_ZERO: set arm 6 joint position and gripper position to zero
    target_joint_position_.fill(0);
    target_gripper_joint_position_ = 0;
    drag_teaching_ = false;
  } else if (mode == 1) {
    // GRAVITY_COMPENSATION: support drag teaching
    drag_teaching_ = true;
  } else {
    // TODO: other control mode
    AWARN << "Invalid control mode. Currently only support 0(GO_ZERO) and "
             "1()GRAVITY_COMPENSATION.";
  }
}

void CanManager::SetArmJointPosition(
    const std::array<double, 6>& arm_joint_position) {
  target_joint_position_ = arm_joint_position;
}

void CanManager::SetArmEndPose(const std::array<double, 6>& arm_end_pose) {
  // TODO: arm Inverse Kinematics compute joint position.

  // target_joint_position_  =
}

void CanManager::SetGripperJointPosition(double gripper_joint_position) {
  target_gripper_joint_position_ = gripper_joint_position;
}

void CanManager::GenerateControlThread() {
  while (!stop_flag_.load()) {
    can_frame write_frame;
    if (drag_teaching_) {
      // TODO: // grivaty compensation(leader arm)

    } else {
      // arm normal control(follower arm)
      for (int i = 0; i < motor_writers_.size(); i++) {
        MitControlMode control_command;
        // position
        if (i < 6) {
          control_command.position = target_joint_position_.at(i);
        } else {
          control_command.position = target_gripper_joint_position_;
        }
        // velocity
        control_command.velocity = 0.0;
        // TODO: need fix kp, kd
        // kp
        control_command.kp = 15;
        // kd
        control_command.kd = 1;
        // torque
        control_command.torque = 0;

        motor_writers_.at(i)->MitControl(write_frame, control_command);
        if (!WriteCanFrame(write_frame, socket_.load())) {
          AERROR << "Failed to write can frame to device: "
                 << motor_writers_.at(i)->name();
          count_fail_ += 1;
        } else {
          count_success_ += 1;
        }
      }
    }

    // AINFO << "count_fail_ : " << count_fail_ << " ,count_success_ : " <<
    // count_success_;
    std::this_thread::sleep_for(std::chrono::milliseconds(4));
  }
}

bool CanManager::OpenCanDevice(const std::string& can_id, int& socket) {
  // create socket
  int s = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (s < 0) {
    AERROR << "Invalid socket can";
    return false;
  }

  // specifying the can device id
  struct ifreq ifr;
  ::strcpy(ifr.ifr_name, can_id.c_str());
  if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
    AERROR << "Failed to ioctrl";
    return false;
  }

  // bind socket to can device
  struct sockaddr_can addr;
  ::memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    AERROR << "Failed to bind";
    return false;
  }

  // set read timeout
  struct timeval timeout;
  timeout.tv_sec = 1;
  timeout.tv_usec = 0;
  setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

  socket = s;

  return true;
}

void CanManager::GenerateReaderThread() {
  // for debug reader data count
  std::vector<int> data_counts(motor_readers_.size(), 0);
  auto start_time = std::chrono::steady_clock::now();

  struct can_frame frame;
  while (!stop_flag_.load()) {
    // read can frame
    int nbytes = ::read(socket_.load(), &frame, sizeof(struct can_frame));
    if (nbytes < 0) {
      AWARN << "read empty can frame";
      continue;
    }

    if (nbytes < sizeof(struct can_frame)) {
      AWARN << "read incomplete can frame";
      continue;
    }

    // parse can frame
    {
      std::lock_guard<std::mutex> lock(reader_mutux_);
      for (int i = 0; i < motor_readers_.size(); i++) {
        if (motor_readers_.at(i)->id() == frame.can_id) {
          if (!motor_readers_.at(i)->ReadCanFrame(frame)) {
            AERROR << "Failed to parse can frame";
          }
          data_counts.at(i)++;

          break;
        }
      }
    }

    // for debug reader data count
    // auto end_time = std::chrono::steady_clock::now();
    // double speed_time =
    // std::chrono::duration_cast<std::chrono::milliseconds>(
    //                         end_time - start_time)
    //                         .count();
    // if (speed_time >= 1000) {
    //   start_time = std::chrono::steady_clock::now();
    //   AINFO << std::this_thread::get_id() << " read data count : ";
    //   for (int i = 0; i < data_counts.size(); i++) {
    //     AINFO << readers.at(i)->name() << ":" << data_counts.at(i) << " ";
    //     data_counts.at(i) = 0;
    //   }
    // }

    // not need set frequence
  }

  AINFO << std::this_thread::get_id() << " thread finish";
}

bool CanManager::WriteCanFrame(const can_frame& frame, int socket) const {
  int bytes = ::write(socket, &frame, sizeof(struct can_frame));
  if (bytes < 0) {
    // 获取错误码
    int error_code = errno;
    // 根据不同的错误码处理不同的情况
    switch (error_code) {
      case EINTR:
        // 写入操作被信号中断
        // 可以考虑重新尝试写入
        break;

      case EAGAIN:  // 或 EWOULDBLOCK (通常相同值)
        // 非阻塞模式下资源暂时不可用
        // 可以稍后重试或等待可写事件
        break;

      case ENETDOWN:
        // 网络接口已关闭
        // 需要重新初始化CAN接口
        break;

      case ENOBUFS:
        // 系统缓冲区不足
        // 可能需要减少发送频率或增加缓冲区
        break;

      case ENXIO:
        // CAN设备可能不存在或未正确配置
        // 检查CAN接口配置
        break;

      case EINVAL:
        // 无效参数，可能是socket或frame有问题
        // 检查socket和frame结构
        break;

      default:
        // 其他未处理的错误
        break;
    }

    AERROR << "Failed to write can frame: " << strerror(errno);

    return false;
  }

  if (bytes < sizeof(struct can_frame)) {
    AERROR << "Write incomplete can frame";
    return false;
  }

  return true;
}

}  // namespace controller
}  // namespace imeta
