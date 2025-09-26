#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "y1_msg/msg/arm_end_pose_control.hpp"
#include "y1_msg/msg/arm_joint_position_control.hpp"
#include "y1_msg/msg/arm_joint_state.hpp"
#include "y1_msg/msg/arm_status.hpp"
#include "y1_sdk/y1_sdk_interface.h"

namespace imeta {
namespace y1_controller {

using namespace std::chrono_literals;

/**
 * @class Y1Controller
 * @brief ros2 interface for y1 controller. (ros2 humble for ubuntu 22.04)
 */
class Y1Controller : public rclcpp::Node {
 public:
  Y1Controller();
  ~Y1Controller() = default;

 public:
  /**
   * @brief init all related class of y1 controller.
   * @result must be true to run normally.
   */
  bool Init();

 private:
  /**
   * @brief follow arm receive leader arm joint state as control command.
   */
  void FollowArmJointPositionControlCallback(
      const y1_msg::msg::ArmJointState::SharedPtr msg);

  /**
   * @brief normal control arm receive end pose control command.
   */
  void ArmEndPoseControlCallback(
      const y1_msg::msg::ArmEndPoseControl::SharedPtr msg);

  /**
   * @brief normal control arm receive joint position control command.
   */
  void ArmJointPositionControlCallback(
      const y1_msg::msg::ArmJointPositionControl::SharedPtr msg);

  /**
   * @brief publish arm joint states at a fixed frequency
   */
  void ArmInformationTimerCallback();

 private:
  rclcpp::Publisher<y1_msg::msg::ArmJointState>::SharedPtr arm_joint_state_pub_;
  rclcpp::Publisher<y1_msg::msg::ArmStatus>::SharedPtr arm_status_pub_;
  rclcpp::Subscription<y1_msg::msg::ArmEndPoseControl>::SharedPtr
      arm_end_pose_control_sub_;
  rclcpp::Subscription<y1_msg::msg::ArmJointState>::SharedPtr
      arm_joint_position_control_sub_;
  rclcpp::Subscription<y1_msg::msg::ArmJointPositionControl>::SharedPtr
      arm_joint_position_control_sub2_;

  rclcpp::TimerBase::SharedPtr arm_information_timer_;

  std::shared_ptr<Y1SDKInterface> y1_interface_;

  // ros params
  int arm_feedback_rate_;
  std::string arm_end_pose_control_topic_;
  std::string arm_joint_position_control_topic_;
  std::string arm_joint_state_topic_;
  std::string arm_status_topic_;
  std::string arm_control_type_;
};

}  // namespace y1_controller
}  // namespace imeta