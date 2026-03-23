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

  // whether is simulation
  is_sim_ = this->declare_parameter("is_sim", false);
  // joint position control topic in simulation
  sim_joint_postion_control_topic_ = this->declare_parameter(
      "sim_joint_position_control_topic", std::string("/joint_states"));

  // leader_arm(master), follower_arm(slave), default is follower_arm
  arm_control_type_ =
      this->declare_parameter("arm_control_type", std::string("follower_arm"));
  // 0: nothing, 1: gripper, 2: teaching pendant, default is 0
  int arm_end_type = this->declare_parameter("arm_end_type", 0);
  // whether to enable robotic arm, default is true
  bool auto_enable = this->declare_parameter("auto_enable", false);

  std::cout << "is_sim_ = " << is_sim_ << std::endl;
  std::cout << "auto_enable = " << auto_enable << std::endl;
  std::cout << "sim_joint_postion_control_topic = " << sim_joint_postion_control_topic_<< std::endl;

  // get urdf path
  std::string package_path =
      ament_index_cpp::get_package_share_directory("y1_controller");
  std::string urdf_path;
  if (arm_end_type == 0) {
    // only load robotic arm
    urdf_path = package_path + "/urdf/y1_no_gripper.urdf";
  } else if (arm_end_type == 1) {
    // robotic arm and gripper_T
    urdf_path = package_path + "/urdf/y1_gripper_t.urdf";
  } else if (arm_end_type == 2) {
    // robotic arm and gripper_G
    urdf_path = package_path + "/urdf/y1_gripper_g.urdf";
  } else if (arm_end_type == 3) {
    // robotic arm and gripper_GT
    urdf_path = package_path + "/urdf/y1_with_gripper.urdf";
  } else {
    RCLCPP_ERROR(this->get_logger(), "arm_end_type is %d , not supported",
                 arm_end_type);
  }

  // Make Y1 SDK Interface
  y1_interface_ = std::make_shared<Y1SDKInterface>(can_id, urdf_path,
                                                   arm_end_type, auto_enable);
}

bool Y1Controller::Init() {
  // init y1 sdk interface
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
    master_arm_joint_position_sub_ =
        this->create_subscription<y1_msg::msg::ArmJointState>(
            arm_joint_position_control_topic_, 1,
            std::bind(&Y1Controller::FollowArmJointPositionControlCallback,
                      this, std::placeholders::_1));

  } else if (arm_control_type_ == "normal_arm") {
    y1_interface_->SetArmControlMode(
        Y1SDKInterface::ControlMode::NRT_JOINT_POSITION);
    // subscriber
    // normal control arm receive control command.
    // arm_end_pose_control_sub_ =
    //     this->create_subscription<y1_msg::msg::ArmEndPoseControl>(
    //         arm_end_pose_control_topic_, 1,
    //         std::bind(&Y1Controller::ArmEndPoseControlCallback, this,
    //                   std::placeholders::_1));
    // arm_joint_position_control_sub_ =
    //     this->create_subscription<y1_msg::msg::ArmJointPositionControl>(
    //         arm_joint_position_control_topic_, 1,
    //         std::bind(&Y1Controller::ArmJointPositionControlCallback, this,
    //                   std::placeholders::_1));
    std::cout << "is_sim_ = " << is_sim_ << std::endl;
    if (is_sim_) {
      sim_joint_position_control_sub_ =
          this->create_subscription<sensor_msgs::msg::JointState>(
              sim_joint_postion_control_topic_, 1,
              std::bind(&Y1Controller::SimPositionControlCallback, this,
                        std::placeholders::_1));
    } else {
      // normal control arm receive control command.
      arm_end_pose_control_sub_ =
          this->create_subscription<y1_msg::msg::ArmEndPoseControl>(
              arm_end_pose_control_topic_, 1,
              std::bind(&Y1Controller::ArmEndPoseControlCallback, this,
                        std::placeholders::_1));
      arm_joint_position_control_sub_ =
          this->create_subscription<y1_msg::msg::ArmJointPositionControl>(
              arm_joint_position_control_topic_, 1,
              std::bind(&Y1Controller::ArmJointPositionControlCallback, this,
                        std::placeholders::_1));
    }

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
  y1_interface_->SetArmEndPose(end_pose);
  // gripper stroke (mm)
  y1_interface_->SetGripperStroke(msg->gripper_stroke);
}

void Y1Controller::FollowArmJointPositionControlCallback(
    const y1_msg::msg::ArmJointState::SharedPtr msg) {
  if (msg->joint_position.size() >= 6) {
    // arm joint position
    y1_interface_->SetArmJointPosition(msg->joint_position);
  } else {
    RCLCPP_ERROR(this->get_logger(),
                 "follow arm receive joint control size < 6");
  }
}

void Y1Controller::ArmJointPositionControlCallback(
    const y1_msg::msg::ArmJointPositionControl::SharedPtr msg) {
  // arm joint position and velocity
  std::array<double, 6> joint_position;
  for (size_t i = 0; i < 6; i++) {
    joint_position[i] = msg->joint_position[i];
  }
  y1_interface_->SetArmJointPosition(joint_position, msg->joint_velocity);

  // gripper stroke (mm)
  y1_interface_->SetGripperStroke(msg->gripper_stroke, msg->gripper_velocity);
}

void Y1Controller::SimPositionControlCallback(
    const sensor_msgs::msg::JointState::ConstSharedPtr &msg) 
{
  
  // 1.创建一个字典 (std::map) 来存储关节名称与位置的映射
  // 对应 Python: joint_positions = {}
  std::map<std::string, double> joint_positions_map;

  // 用于存储夹爪关节的值
  double gripper_pos_raw = 0.0;
  bool gripper_found = false;

  // 2. 遍历 msg->name 来映射位置
  if (msg->name.size() != msg->position.size()) {
    RCLCPP_ERROR(this->get_logger(), "JointState name and position size mismatch!");
    return;
  }

  for (size_t i = 0; i < msg->name.size(); ++i) {
    const std::string& name = msg->name[i];
    double pos = msg->position[i];

    // 存入字典
    joint_positions_map[name] = pos;
    
    // 方案：优先信任索引6是夹爪（兼容旧逻辑），但也记录名字以便调试
    if (i == 6) {
      gripper_pos_raw = msg->position[i];
      gripper_found = true;
    }
    
    // 额外保险：如果有关节名叫 "gripper" 或 "joint7"，也视为夹爪
    if (name.find("gripper") != std::string::npos || name == "joint7") {
       gripper_pos_raw = msg->position[i];
       gripper_found = true;
    }
  }

  // 3. 使用关节名称来动态控制关节 (核心修复部分)
  // 构建一个顺序正确的数组，确保索引0是joint1，索引1是joint2...
  std::array<double, 6> arm_joint_position;
  
  // 定义期望的关节名称顺序
  const std::vector<std::string> expected_names = {
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"
  };

  // bool all_joints_found = true;
  for (int i = 0; i < 6; ++i) {
    const std::string& target_name = expected_names[i];
    
    // 在 map 中查找
    auto it = joint_positions_map.find(target_name);
    if (it != joint_positions_map.end()) {
      arm_joint_position[i] = it->second;
    } else {
      // 如果找不到该关节，报错并使用 0 (或者保持上一时刻的值，视安全策略而定)
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                           "Joint '%s' not found in JointState message!", target_name.c_str());
      arm_joint_position[i] = 0.0; 
      // all_joints_found = false; // 如果缺少关节必须停止运动，可取消注释并添加返回逻辑
    }
  }

  // 4. 下发机械臂控制指令
  // 此时 arm_joint_position 的顺序已经强制对齐为 joint1~joint6
  y1_interface_->SetArmJointPosition(arm_joint_position, 6);

  // 5. 夹爪控制
  if (gripper_found) {
    double gripper_stroke = -gripper_pos_raw * 2000.0;
    
    // 可选：添加类似 Python 的 clip 或 NaN 检查
    if (std::isnan(gripper_stroke)) {
      gripper_stroke = 0.0; // 默认值
      RCLCPP_WARN(this->get_logger(), "Gripper position is NaN, using default.");
    }

    y1_interface_->SetGripperStroke(gripper_stroke, 6);
  }
}

void Y1Controller::ArmInformationTimerCallback() {
  // robotic arm joint state
  y1_msg::msg::ArmJointState arm_joint_state;
  // auto arm_joint_state = y1_msg::msg::ArmJointState();
  arm_joint_state.header.stamp = this->now();

  // get arm end pose
  std::array<double, 6> arm_end_pose = y1_interface_->GetArmEndPose();

  // get 6 or 7(include gripper) joint position.
  std::vector<double> joint_position = y1_interface_->GetJointPosition();

  // get 6 or 7(include gripper) joint velocity.
  std::vector<double> joint_velocity = y1_interface_->GetJointVelocity();

  // get 6 or 7(include gripper) joint torque.
  std::vector<double> joint_effort = y1_interface_->GetJointEffort();

  for (size_t i = 0; i < joint_position.size(); i++) {
    arm_joint_state.joint_position.push_back(joint_position.at(i));
    arm_joint_state.joint_velocity.push_back(joint_velocity.at(i));
    arm_joint_state.joint_effort.push_back(joint_effort.at(i));
  }

  for (size_t i = 0; i < 6; i++) {
    arm_joint_state.end_pose.at(i) = arm_end_pose.at(i);
  }

  // joint motor status
  y1_msg::msg::ArmStatus arm_status;
  arm_status.header.stamp = this->now();

  // get joint names
  std::vector<std::string> joint_names = y1_interface_->GetJointNames();

  // get motor current
  std::vector<double> motor_current = y1_interface_->GetMotorCurrent();

  // get rotor temperature
  std::vector<double> rotor_temperature = y1_interface_->GetRotorTemperature();

  // get joint motor error code
  std::vector<int> joint_error_code = y1_interface_->GetJointErrorCode();

  double total_current = 0;
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

  // publish arm information
  arm_joint_state_pub_->publish(arm_joint_state);
  arm_status_pub_->publish(arm_status);
}

}  // namespace y1_controller
}  // namespace imeta