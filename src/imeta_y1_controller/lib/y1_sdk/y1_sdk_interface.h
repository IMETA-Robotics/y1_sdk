#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace imeta {
namespace controller {

class Y1SDKInterface {
 public:
  Y1SDKInterface(const std::string& can_id, const std::string& urdf_path,
                 int arm_end_type, bool enable_arm);
  ~Y1SDKInterface();

  /**
   * @brief must be initialize the SDK interface.
   * @return if the SDK interface is initialized successfully.
   */
  bool Init();

  enum ControlMode { GO_ZERO = 0, GRAVITY_COMPENSATION = 1 };

  /**
   * @brief the interface of joint position.
   * @return 6 or 7(include gripper) joint position.
   */
  std::vector<double> GetJointPosition();

  /**
   * @brief the interface of joint velocity.
   * @return 6 or 7(include gripper) joint velocity.
   */
  std::vector<double> GetJointVelocity();

  /**
   * @brief the interface of joint toruqe.
   * @return 6 or 7(include gripper) joint toruqe.
   */
  std::vector<double> GetJointEffort();

  /**
   * @brief the interface of arm end pose.
   * @return 6 size (x y z roll pitch yaw)
   */
  std::array<double, 6> GetArmEndPose();

  /**
   * @brief set arm control mode. (0: go_zero, 1: gravity compensation)
   */
  void SetArmControlMode(const ControlMode& mode);

  /**
   * @brief set arm joint position control command.
   */
  void SetArmJointPosition(const std::array<double, 6>& arm_joint_position);

  /**
   * @brief set arm end pose control command. (x y z roll pitch yaw)
   */
  void SetArmEndPose(const std::array<double, 6>& arm_end_pose);

  /**
   * @brief set gripper joint position control command.
   */
  void SetGripperJointPosition(double gripper_joint_position);

  /**
   * @brief Enable or disable the motor of the robot arm.
   * @param enable_arm true: enable the motor, false: disable the motor.
   */
  void SetEnableArm(bool enable_flag);

  /**
   * @brief Save all joint zero position.
      J1 - J5 have been saved with zero point when leaving the factory.
  */
  // void SaveAllJointZeroPosition();

  /**
   * @brief Save J6 joint zero position.
      Each time you reinstall the end gripper or the teach pendant, you need to
   set the zero point of J6.
  */
  void SaveJ6ZeroPosition();

 private:
  class Impl;
  std::unique_ptr<Impl> pimpl_;
};

}  // namespace controller
}  // namespace imeta