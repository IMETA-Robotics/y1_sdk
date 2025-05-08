#include "y1_sdk_interface.h"

#include <memory>
#include <string>

#include "can_manager.h"

namespace imeta {
namespace controller {

class Y1SDKInterface::Impl {
 public:
  explicit Impl(const std::string& can_id, const std::string& urdf_path,
                int arm_end_type)
      : can_id_(can_id),
        urdf_path_(urdf_path),
        can_manager_(
            std::make_unique<CanManager>(can_id, urdf_path, arm_end_type)) {}
  ~Impl() = default;

  /**
   * @brief must be initialize the SDK interface.
   * @return if the SDK interface is initialized successfully.
   */
  bool Init() {
    return can_manager_->Init();
  }

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
  std::vector<double> GetArmEndPose();

  /**
   * @brief set arm control mode. (1:joint position control, 2:end pose control,
   * 3: gravity compensation)
   */
  void SetArmControlMode(int mode);

  /**
   * @brief set arm joint position control command.
   */
  void SetArmJointPosition(const std::vector<double>& arm_joint_position);

  /**
   * @brief set arm end pose control command. (x y z roll pitch yaw)
   */
  // TODO: std::vector to std::array?
  void SetArmEndPose(const std::vector<double>& arm_end_pose);

  /**
   * @brief set gripper joint position control command.
   */
  void SetGripperJointPosition(double gripper_joint_position);

 private:
  std::string can_id_;
  std::string urdf_path_;

  std::unique_ptr<CanManager> can_manager_;
};

Y1SDKInterface::Y1SDKInterface(const std::string& can_id,
                               const std::string& urdf_path, int arm_end_type)
    : pimpl_(std::make_unique<Impl>(can_id, urdf_path, arm_end_type)) {}

bool Y1SDKInterface::Init() {
  if (!pimpl_->Init()) {
    return false;
  }

  return true;
}

std::vector<double> Y1SDKInterface::GetJointPosition() {
  return pimpl_->GetJointPosition();
}

std::vector<double> Y1SDKInterface::GetJointVelocity() {
  return pimpl_->GetJointVelocity();
}

std::vector<double> Y1SDKInterface::GetJointEffort() {
  return pimpl_->GetJointEffort();
}

std::vector<double> Y1SDKInterface::GetArmEndPose() {
  return pimpl_->GetArmEndPose();
}

void Y1SDKInterface::SetArmControlMode(const ControlMode& mode) {
  pimpl_->SetArmControlMode(static_cast<int>(mode));
}

void Y1SDKInterface::SetArmJointPosition(
    const std::vector<double>& arm_joint_position) {
  pimpl_->SetArmJointPosition(arm_joint_position);
}

// TODO: std::vector to std::array?
void Y1SDKInterface::SetArmEndPose(const std::vector<double>& arm_end_pose) {
  pimpl_->SetArmEndPose(arm_end_pose);
}

void Y1SDKInterface::SetGripperJointPosition(double gripper_joint_position) {
  pimpl_->SetGripperJointPosition(gripper_joint_position);
}

}  // namespace controller
}  // namespace imeta