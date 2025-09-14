#pragma once

#include <ros/ros.h>

#include <memory>
#include <string>

#include "y1_msg/ArmEndPoseControl.h"
#include "y1_msg/ArmJointPositionControl.h"
#include "y1_msg/ArmJointState.h"
#include "ros/subscriber.h"
#include "y1_sdk/y1_sdk_interface.h"

namespace imeta {
namespace y1_controller {

/**
 * @class Y1Controller
 * @brief ros1 interface for y1 controller. (ros1 noetic for ubuntu 20.04)
 */
class Y1Controller {
 public:
  Y1Controller() : nh_("~") {}
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
      const y1_msg::ArmJointState::ConstPtr& msg);

  /**
   * @brief normal control arm receive end pose control command.
   */
  void ArmEndPoseControlCallback(
      const y1_msg::ArmEndPoseControl::ConstPtr& msg);

  /**
   * @brief normal control arm receive joint position control command.
   */
  void ArmJointPositionControlCallback(
      const y1_msg::ArmJointPositionControl::ConstPtr& msg);

  /**
   * @brief publish arm joint states at a fixed frequency
   */
  void ArmInformationTimerCallback(const ros::TimerEvent&);

 private:
  ros::NodeHandle nh_;
  ros::Publisher arm_joint_state_pub_;
  ros::Publisher arm_status_pub_;
  ros::Subscriber arm_end_pose_control_sub_;
  ros::Subscriber arm_joint_position_control_sub_;

  ros::Timer arm_information_timer_;

  std::shared_ptr<Y1SDKInterface> y1_interface_;
};

}  // namespace y1_controller
}  // namespace imeta
