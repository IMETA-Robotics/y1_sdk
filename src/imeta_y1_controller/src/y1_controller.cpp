#include "y1_controller.h"

#include <ros/package.h>

#include "imeta_y1_msg/ArmControl.h"
#include "imeta_y1_msg/ArmState.h"

namespace imeta {
namespace controller {
bool Y1Controller::Init() {
  // init ros parameters
  // InitParams();
  std::string can_id = nh_.param("arm_can_id", std::string("can1"));
  int arm_feedback_rate = nh_.param("arm_feedback_rate", 200);
  std::string arm_control_topic =
      nh_.param("arm_control_topic", std::string("/y1_controller/arm_control"));
  std::string arm_state_topic =
      nh_.param("arm_feedback_topic", std::string("/y1_controller/arm_state"));
  // leader_arm(master), follower_arm(slave), default is follower_arm
  std::string arm_control_type =
      nh_.param("arm_control_type", std::string("follower_arm"));
  // 0: nothing, 1: gripper, 2: teaching pendant, default is 0
  int arm_end_type = nh_.param("arm_end_type", 0);

  // get urdf path
  // 暂时用松灵Piper的urdf测试
  std::string package_path = ros::package::getPath("imeta_y1_controller");
  std::string urdf_path;
  if (arm_end_type == 0) {
    // only load robotic arm
    urdf_path = package_path + "/urdf/y1_arm_no_gripper.urdf";
  } else if (arm_end_type == 1) {
    urdf_path = package_path + "/urdf/piper_description.urdf";
  } else if (arm_end_type == 2) {
    urdf_path = package_path + "/urdf/y1_arm_with_teaching_pendant.urdf";
  } else {
    ROS_ERROR("arm_end_type is %d , not supported", arm_end_type);
    return false;
  }

  // TODO: init Y1 SDK Interface
  y1_interface_ =
      std::make_unique<Y1SDKInterface>(can_id, urdf_path, arm_end_type);
  if (!y1_interface_->Init()) {
    ROS_ERROR("Init Y1 SDK Interface failed.");
    return false;
  }

  if (arm_control_type == "leader_arm") {
    // leader arm need gravity compensation
    y1_interface_->SetArmControlMode(
        Y1SDKInterface::ControlMode::GRAVITY_COMPENSATION);

  } else if (arm_control_type == "follower_arm") {
    // subscriber. follower arm receive control command.
    arm_control_sub_ = nh_.subscribe(arm_control_topic, 1,
                                     &Y1Controller::ArmControlCallback, this);

  } else {
    ROS_ERROR("arm_control_type is %s , not supported",
              arm_control_type.c_str());
    return false;
  }

  // publisher
  arm_state_pub_ = nh_.advertise<imeta_y1_msg::ArmState>(arm_state_topic, 1);

  // publish arm joint states at a fixed frequency
  arm_state_timer_ =
      nh_.createTimer(ros::Duration(1.0 / arm_feedback_rate),
                      &Y1Controller::ArmStateTimerCallback, this);

  ROS_INFO("y1_controller success start!");

  return true;
}

void Y1Controller::ArmControlCallback(
    const imeta_y1_msg::ArmControl::ConstPtr& msg) {
  // TODO: end pose or joint position control arm
}

void Y1Controller::ArmStateTimerCallback(const ros::TimerEvent&) {
  imeta_y1_msg::ArmState arm_state;
  // TODO: publish arm state
}

}  // namespace controller
}  // namespace imeta