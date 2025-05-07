#include "can_manager.h"

#include <linux/can.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <chrono>
#include <cstring>

#include "can_readers/dm_motor_reader.h"
#include "can_writers/dm_motor_writer.h"
#include "common/log.h"

namespace humanoid {
namespace can_driver {

CanManager::CanManager(const CanDriverConfig& config) : config_(config) {}

CanManager::~CanManager() {
  stop_flag_.store(true);

  // Disable motor
  can_frame frame;
  for (int i = 0; i < can_writers_.size(); i++) {
    can_writers_.at(i)->Disable(frame);
    if (!WriteCanFrame(frame, can_writers_.at(i)->socket())) {
      AERROR << "Failed to disable device: " << can_writers_.at(i)->name();
    }
  }

  // wait for all threads finish
  for (auto& thread : reader_threads_) {
    if (thread.joinable()) {
      AINFO << std::this_thread::get_id() << " thread join";
      thread.join();
    }
  }
}

bool CanManager::Init() {
  if (config_.can_size() == 0) {
    AERROR << "can device number is 0 in can_driver_config";
    return false;
  }

  // open can device
  for (int i = 0; i < config_.can_size(); i++) {
    auto can_device = config_.can(i);
    int socket;
    if (!OpenCanDevice(can_device.can_device_id(), socket)) {
      AERROR << "Failed to open can device:" << can_device.can_device_id();
      return false;
    }

    if (can_device.device_info_size() == 0) {
      AERROR << can_device.can_device_id() << " 's device number is 0";
      return false;
    }

    std::vector<std::shared_ptr<CanReaderBase>> readers;
    for (int i = 0; i < can_device.device_info_size(); i++) {
      // readers
      auto device_read_info = can_device.device_info(i).device_read_info();
      if (device_read_info.type() == "DmMotorReader") {
        std::shared_ptr<CanReaderBase> reader =
            std::make_shared<DmMotorReader>();
        reader->Init(device_read_info.name(), device_read_info.id());

        readers.emplace_back(reader);
      } else {
        AERROR << "Failed create " << device_read_info.type();
        return false;
      }

      // writers
      auto device_write_info = can_device.device_info(i).device_write_info();
      if (device_write_info.type() == "DmMotorWriter") {
        std::unique_ptr<CanWriterBase> writer =
            std::make_unique<DmMotorWriter>(can_device.device_info(i));
        writer->Init(device_read_info.name(), device_write_info.id(), socket);

        can_writers_.emplace_back(std::move(writer));
      } else {
        AERROR << "Failed create " << device_write_info.type();
        return false;
      }
    }

    // save readers every can
    can_readers_.emplace_back(readers);

    // generate one thread for every can
    auto mtx = std::make_shared<std::mutex>();
    readers_mutux_.emplace_back(mtx);
    reader_threads_.emplace_back(&CanManager::GenerateThread, this, readers,
                                 socket, mtx);
  }

  // motor enable
  can_frame frame;
  for (int i = 0; i < can_writers_.size(); i++) {
    can_writers_.at(i)->Enable(frame);
    if (!WriteCanFrame(frame, can_writers_.at(i)->socket())) {
      AERROR << "Failed to enable device: " << can_writers_.at(i)->name();
      return false;
    }
  }

  return true;
}

void CanManager::RunOnce(const ControlCommandVector& control_command,
                         MotorStateVector& motor_states_info) {
  // writer
  // control command number can bigger than motor device number
  ACHECK(control_command.size() >= can_writers_.size())
      << "Control Command size < Motor Writer size ";

  can_frame frame;
  for (int i = 0; i < can_writers_.size(); i++) {
    can_writers_.at(i)->WriteCanFrame(frame, control_command.at(i));
    if (!WriteCanFrame(frame, can_writers_.at(i)->socket())) {
      AERROR << "Failed to write can frame to device: "
             << can_writers_.at(i)->name();
      count_fail_ += 1;
    } else {
      count_success_ += 1;
    }
  }

  // reader
  for (int i = 0; i < can_readers_.size(); i++) {
    {
      // protect socket
      std::lock_guard<std::mutex> lock(*readers_mutux_.at(i));
      auto readers = can_readers_.at(i);
      for (int j = 0; j < readers.size(); j++) {
        motor_states_info.emplace_back(readers.at(j)->motor_state());
      }
    }
  }

  // AINFO << "count_fail_ : " << count_fail_ << " ,count_success_ : " <<
  // count_success_;
}

bool CanManager::OpenCanDevice(const std::string& can_device_id, int& socket) {
  // create socket
  int s = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (s < 0) {
    AERROR << "Invalid socket can";
    return false;
  }

  // specifying the can device id
  struct ifreq ifr;
  ::strcpy(ifr.ifr_name, can_device_id.c_str());
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

  // save socket
  socket_can_map_.emplace(can_device_id, s);
  socket = s;

  return true;
}

void CanManager::GenerateThread(
    const std::vector<std::shared_ptr<CanReaderBase>>& readers, int socket,
    std::shared_ptr<std::mutex> mtx) {
  // for debug reader data count
  std::vector<int> data_counts(readers.size(), 0);
  auto start_time = std::chrono::steady_clock::now();

  struct can_frame frame;
  while (!stop_flag_.load()) {
    // read can frame
    int nbytes = ::read(socket, &frame, sizeof(struct can_frame));
    if (nbytes < 0) {
      AWARN << "read empty can frame";
      continue;
    }

    if (nbytes < sizeof(struct can_frame)) {
      AWARN << "read incomplete can frame";
      continue;
    }

    // AINFO << std::this_thread::get_id() << " thread : " << nbytes;

    // parse can frame
    {
      std::lock_guard<std::mutex> lock(*mtx);
      for (int i = 0; i < readers.size(); i++) {
        if (readers.at(i)->id() == frame.can_id) {
          if (!readers.at(i)->ReadCanFrame(frame)) {
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

}  // namespace can_driver
}  // namespace humanoid
