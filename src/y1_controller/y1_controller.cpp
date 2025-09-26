#include "y1_controller.h"

// #include <glog/logging.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "std_msgs/msg/string.hpp"
#include "y1_msg/msg/arm_status.hpp"

namespace imeta {
namespace y1_controller {

Y1Controller::Y1Controller() : Node("y1_controller") {
  // init ros parameters
  std::string can_id =
      this->declare_parameter("arm_can_id", std::string("can0"));
  arm_feedback_rate_ = this->declare_parameter("arm_feedback_rate", 200);

  // end pose control mode
  arm_end_pose_control_topic_ = this->declare_parameter(
      "arm_end_pose_control_topic", std::string("/y1/arm_end_pose_control"));
  // joint position control mode
  arm_joint_position_control_topic_ = this->declare_parameter(
      "arm_joint_position_control_topic",
      std::string("/y1/arm_joint_position_control_topic"));
  // joint state feedback
  arm_joint_state_topic_ = this->declare_parameter(
      "arm_joint_state_topic", std::string("/y1/arm_joint_state"));
  // joint motor status feedback
  arm_status_topic_ = this->declare_parameter("arm_status_topic",
                                              std::string("/y1/arm_status"));

  // leader_arm(master), follower_arm(slave), default is follower_arm
  arm_control_type_ =
      this->declare_parameter("arm_control_type", std::string("follower_arm"));
  // 0: nothing, 1: gripper, 2: teaching pendant, default is 0
  int arm_end_type = this->declare_parameter("arm_end_type", 0);
  // whether to enable robotic arm, default is true
  bool auto_enable = this->declare_parameter("auto_enable", true);

  // get urdf path
  std::string package_path =
      ament_index_cpp::get_package_share_directory("y1_controller");
  std::string urdf_path;
  if (arm_end_type == 0) {
    // only load robotic arm
    urdf_path = package_path + "/urdf/y10804.urdf";
  } else if (arm_end_type == 1) {
    // robotic arm and gripper_T
    urdf_path = package_path + "/urdf/y1_gripper_t.urdf";
  } else if (arm_end_type == 2) {
    // robotic arm and gripper_G
    urdf_path = package_path + "/urdf/y1_gripper_g.urdf";
  } else if (arm_end_type == 3) {
    // robotic arm and gripper_GT
    urdf_path = package_path + "/urdf/y10824_ee.urdf";
  } else {
    RCLCPP_ERROR(this->get_logger(), "arm_end_type is %d , not supported",
                 arm_end_type);
    // return false;
  }

  // init Y1 SDK Interface
  y1_interface_ = std::make_shared<Y1SDKInterface>(can_id, urdf_path,
                                                   arm_end_type, auto_enable);
}

bool Y1Controller::Init() {
  if (!y1_interface_->Init()) {
    RCLCPP_ERROR(this->get_logger(), "Init Y1 SDK Interface failed.");
    return false;
  }

  if (arm_control_type_ == "leader_arm") {
    // leader arm need gravity compensation
    y1_interface_->SetArmControlMode(
        Y1SDKInterface::ControlMode::GRAVITY_COMPENSATION);

  } else if (arm_control_type_ == "follower_arm") {
    y1_interface_->SetArmControlMode(
        Y1SDKInterface::ControlMode::RT_JOINT_POSITION);
    // subscriber. follower arm receive leader arm joint state as control
    // command.
    arm_end_pose_control_sub_ =
        this->create_subscription<y1_msg::msg::ArmEndPoseControl>(
            arm_end_pose_control_topic_, 1,
            std::bind(&Y1Controller::ArmEndPoseControlCallback, this,
                      std::placeholders::_1));
    arm_joint_position_control_sub_ =
        this->create_subscription<y1_msg::msg::ArmJointState>(
            arm_joint_position_control_topic_, 1,
            std::bind(&Y1Controller::FollowArmJointPositionControlCallback,
                      this, std::placeholders::_1));

  } else if (arm_control_type_ == "normal_arm") {
    y1_interface_->SetArmControlMode(
        Y1SDKInterface::ControlMode::NRT_JOINT_POSITION);
    // subscriber
    // normal control arm receive control command.
    arm_end_pose_control_sub_ =
        this->create_subscription<y1_msg::msg::ArmEndPoseControl>(
            arm_end_pose_control_topic_, 1,
            std::bind(&Y1Controller::ArmEndPoseControlCallback, this,
                      std::placeholders::_1));
    arm_joint_position_control_sub2_ =
        this->create_subscription<y1_msg::msg::ArmJointPositionControl>(
            arm_joint_position_control_topic_, 1,
            std::bind(&Y1Controller::ArmJointPositionControlCallback, this,
                      std::placeholders::_1));

  } else {
    RCLCPP_ERROR(this->get_logger(), "arm_control_type is %s , not supported",
                 arm_control_type_.c_str());
    return false;
  }

  // joint states publisher
  arm_joint_state_pub_ = this->create_publisher<y1_msg::msg::ArmJointState>(
      arm_joint_state_topic_, 1);
  // joint motor status publisher
  arm_status_pub_ =
      this->create_publisher<y1_msg::msg::ArmStatus>(arm_status_topic_, 1);

  // publish arm joint states at a fixed frequency
  arm_information_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000 / arm_feedback_rate_),
      std::bind(&Y1Controller::ArmInformationTimerCallback, this));

  RCLCPP_INFO(this->get_logger(), "y1_controller success start!");

  return true;
}

void Y1Controller::ArmEndPoseControlCallback(
    const y1_msg::msg::ArmEndPoseControl::SharedPtr msg) {
  // end pose
  std::array<double, 6> end_pose;
  for (size_t i = 0; i < 6; i++) {
    end_pose[i] = msg->end_pose[i];
  }
  // LOG(INFO) << "TETS1";
  y1_interface_->SetArmEndPose(end_pose);
  // LOG(INFO) << "TETS2";
  // gripper stroke (mm)
  y1_interface_->SetGripperStroke(msg->gripper_stroke);
  // LOG(INFO) << "TETS3";
}

void Y1Controller::FollowArmJointPositionControlCallback(
    const y1_msg::msg::ArmJointState::SharedPtr msg) {
  if (msg->joint_position.size() >= 6 && msg->joint_velocity.size() >= 6) {
    // arm joint position
    y1_interface_->SetArmJointPosition(msg->joint_position);
  } else {
    RCLCPP_ERROR(this->get_logger(),
                 "follow arm receive joint control size < 6");
  }
}

void Y1Controller::ArmJointPositionControlCallback(
    const y1_msg::msg::ArmJointPositionControl::SharedPtr msg) {
  // arm joint position
  std::array<double, 6> joint_position;
  for (size_t i = 0; i < 6; i++) {
    joint_position[i] = msg->joint_position[i];
  }
  y1_interface_->SetArmJointPosition(joint_position);

  // gripper stroke (mm)
  y1_interface_->SetGripperStroke(msg->gripper_stroke);
}

void Y1Controller::ArmInformationTimerCallback() {
  // robotic arm joint state
  auto arm_joint_state = y1_msg::msg::ArmJointState();
  arm_joint_state.header.stamp = this->now();

  // LOG(INFO) << "TETS1";
  // get arm end pose
  std::array<double, 6> arm_end_pose = y1_interface_->GetArmEndPose();

  // LOG(INFO) << "TETS2";
  // get 6 or 7(include gripper) joint position.
  std::vector<double> joint_position = y1_interface_->GetJointPosition();

  // LOG(INFO) << "TETS3";
  // get 6 or 7(include gripper) joint velocity.
  std::vector<double> joint_velocity = y1_interface_->GetJointVelocity();

  // LOG(INFO) << "TETS4";
  // get 6 or 7(include gripper) joint torque.
  std::vector<double> joint_effort = y1_interface_->GetJointEffort();

  // LOG(INFO) << "TETS5";
  for (size_t i = 0; i < joint_position.size(); i++) {
    arm_joint_state.joint_position.push_back(joint_position.at(i));
    arm_joint_state.joint_velocity.push_back(joint_velocity.at(i));
    arm_joint_state.joint_effort.push_back(joint_effort.at(i));
  }

  for (size_t i = 0; i < 6; i++) {
    arm_joint_state.end_pose.at(i) = arm_end_pose.at(i);
  }
  // LOG(INFO) << "TETS6";

  // joint motor status
  auto arm_status = y1_msg::msg::ArmStatus();
  arm_status.header.stamp = this->now();

  // LOG(INFO) << "TETS7";
  // get joint names
  std::vector<std::string> joint_names = y1_interface_->GetJointNames();

  // LOG(INFO) << "TETS8";
  // get motor current
  std::vector<double> motor_current = y1_interface_->GetMotorCurrent();

  // LOG(INFO) << "TETS9";
  // get rotor temperature
  std::vector<double> rotor_temperature = y1_interface_->GetRotorTemperature();

  // LOG(INFO) << "TETS10";
  // get joint motor error code
  std::vector<int> joint_error_code = y1_interface_->GetJointErrorCode();

  double total_current = 0;
  // LOG(INFO) << "TETS11";
  for (size_t i = 0; i < motor_current.size(); i++) {
    std_msgs::msg::String joint_name;
    joint_name.data = joint_names.at(i);
    arm_status.name.push_back(joint_name);
    arm_status.motor_current.push_back(motor_current.at(i));
    arm_status.rotor_temperature.push_back(rotor_temperature.at(i));
    arm_status.error_code.push_back(joint_error_code.at(i));
    total_current += motor_current.at(i);
  }
  arm_status.motor_current.push_back(total_current);
  // LOG(INFO) << "TETS12";

  // publish arm information
  arm_joint_state_pub_->publish(arm_joint_state);
  arm_status_pub_->publish(arm_status);
  // LOG(INFO) << "TETS13";
}

}  // namespace y1_controller
}  // namespace imeta