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
                 int arm_end_type);
  ~Y1SDKInterface() = default;

  /**
   * @brief must be initialize the SDK interface.
   * @return if the SDK interface is initialized successfully.
   */
  bool Init();

  enum ControlMode {
    GO_ZERO = 0,
    JOINT_POSITION_CONTROL = 1,
    END_POSE_CONTROL = 2,
    GRAVITY_COMPENSATION = 3
  };

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
   * @brief set arm control mode. (1:joint position control, 2:end pose control,
   * 3: gravity compensation)
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

 private:
  class Impl;
  std::unique_ptr<Impl> pimpl_;
};

}  // namespace controller
}  // namespace imeta