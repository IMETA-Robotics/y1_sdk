#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace imeta {
namespace controller {

class Y1SDKInterface {
 public:
  Y1SDKInterface() = delete;
  explicit Y1SDKInterface(const std::string& can_id,
                          const std::string& urdf_path, int arm_end_type,
                          bool enable_arm);
  ~Y1SDKInterface();

  /**
   * @brief must be initialize the SDK interface.
   * @return if the SDK interface is initialized successfully.
   */
  bool Init();

  enum ControlMode {
    GRAVITY_COMPENSATION = 0,
    RT_JOINT_POSITION = 1,
    NRT_JOINT_POSITION
  };

  /**
   * @brief the interface of arm all joint names.
   * @return 6 or 7(include gripper) joint names.
   */
  std::vector<std::string> GetJointNames();

  /**
   * @brief the interface of all motor internal coil temperature
   * @return 6 or 7(include gripper) joint rotor temperature.
   */
  std::vector<double> GetRotorTemperature();

  /**
   * @brief the interface of all joint error code.
   * @return 6 or 7(include gripper) joint error code.
   */
  std::vector<int> GetJointErrorCode();

  /**
   * @brief the interface of all motor current.
   * @return 6 or 7(include gripper) motor current.
   */
  std::vector<double> GetMotorCurrent();

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
   * @brief set arm control mode. (0: GRAVITY_COMPENSATION, 1:
   * RT_JOINT_POSITION, 2: NRT_JOINT_POSITION)
   */
  void SetArmControlMode(const ControlMode& mode);

  /**
   * @brief set normal control arm joint position control command.
   */
  void SetArmJointPosition(const std::array<double, 6>& arm_joint_position);

  /**
   * @brief set normal control arm joint velocity control command.
   */
  void SetArmJointVelocity(double arm_joint_velocity);

  /**
   * @brief set normal control arm joint position and velocity control command.
   */
  void ControlArmJoint(const std::array<double, 6>& arm_joint_position,
                       double velocity);

  /**
   * @brief set follow arm joint position control command.
   */
  void SetArmJointPosition(const std::vector<double>& arm_joint_position);

  /**
   * @brief set follow arm joint velocity control command.
   */
  void SetArmJointVelocity(const std::vector<double>& arm_joint_velocity);

  /**
   * @brief set arm end pose control command. (x y z roll pitch yaw)
   */
  void SetArmEndPose(const std::array<double, 6>& arm_end_pose);

  /**
   * @brief set gripper stroke control command. Unit: mm
   */
  void SetGripperStroke(double gripper_stroke);

  /**
   * @brief set gripper stroke and velocity control command.
   * @param gripper_stroke Unit: mm
   * @param velocity Unit: rad/s
   */
  void ControlGripper(double gripper_stroke, double velocity);

  /**
   * @brief Enable or disable the motor of the robot arm.
   * @param enable_arm true: enable the motor, false: disable the motor.
   */
  void SetEnableArm(bool enable_flag);

  /**
   * @brief Save J6 joint zero position.
      Each time you reinstall the end flange adapter, you need to
   set the zero point of J6.
  */
  void SaveJ6ZeroPosition();

 private:
  class Impl;
  std::unique_ptr<Impl> pimpl_;
};

}  // namespace controller
}  // namespace imeta