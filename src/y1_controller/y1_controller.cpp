#include "y1_controller.h"

#include <glog/logging.h>
#include <ros/package.h>

#include <vector>

#include "std_msgs/String.h"
#include "y1_msg/ArmStatus.h"
#include "y1_msg/InteractionForce.h"

namespace imeta {
namespace y1_controller {
bool Y1Controller::Init() {
  // init ros parameters
  std::string can_id = nh_.param("arm_can_id", std::string("can1"));
  int arm_feedback_rate = nh_.param("arm_feedback_rate", 200);

  // end pose control mode
  std::string arm_end_pose_control_topic = nh_.param(
      "arm_end_pose_control_topic", std::string("/y1/arm_end_pose_control"));
  // joint position control mode
  std::string arm_joint_position_control_topic =
      nh_.param("arm_joint_position_control_topic",
                std::string("/y1/arm_joint_position_control_topic"));
  // joint state feedback
  std::string arm_joint_state_topic =
      nh_.param("arm_joint_state_topic", std::string("/y1/arm_joint_state"));
  // joint motor status feedback
  std::string arm_status_topic =
      nh_.param("arm_status_topic", std::string("/y1/arm_status"));

  // leader_arm(master), follower_arm(slave), default is follower_arm
  std::string arm_control_type =
      nh_.param("arm_control_type", std::string("follower_arm"));
  // 0: nothing, 1: gripper, 2: teaching pendant, default is 0
  int arm_end_type = nh_.param("arm_end_type", 0);
  // whether to enable robotic arm, default is true
  bool auto_enable = nh_.param("auto_enable", true);

  // get urdf path
  std::string package_path = ros::package::getPath("y1_controller");
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
    // urdf_path = package_path + "/urdf/y1_gripper_gt.urdf";
    urdf_path = package_path + "/urdf/y10824_ee.urdf";
  } else {
    ROS_ERROR("arm_end_type is %d , not supported", arm_end_type);
    return false;
  }

  // init Y1 SDK Interface
  y1_interface_ = std::make_shared<Y1SDKInterface>(can_id, urdf_path,
                                                   arm_end_type, auto_enable);
  if (!y1_interface_->Init()) {
    ROS_ERROR("Init Y1 SDK Interface failed.");
    return false;
  }

  if (arm_control_type == "leader_arm") {
    // leader arm need gravity compensation
    y1_interface_->SetArmControlMode(
        Y1SDKInterface::ControlMode::GRAVITY_COMPENSATION);
    slave_arm_interaction_sub_ =
        nh_.subscribe("/y1/slave_arm_interaction", 1,
                      &Y1Controller::SlaveArmInteractionCallback, this);
  } else if (arm_control_type == "follower_arm") {
    y1_interface_->SetArmControlMode(
        Y1SDKInterface::ControlMode::RT_JOINT_POSITION);
    // subscriber. follower arm receive leader arm joint state as control
    // command.
    arm_end_pose_control_sub_ =
        nh_.subscribe(arm_end_pose_control_topic, 1,
                      &Y1Controller::ArmEndPoseControlCallback, this);
    arm_joint_position_control_sub_ = nh_.subscribe(
        arm_joint_position_control_topic, 1,
        &Y1Controller::FollowArmJointPositionControlCallback, this);

    slave_arm_interaction_pub_ = nh_.advertise<y1_msg::InteractionForce>(
        "/y1/slave_arm_interaction", 1);
    // publish slave arm interaction force
    slave_arm_interaction_timer_ =
        nh_.createTimer(ros::Duration(1.0 / arm_feedback_rate),
                        &Y1Controller::SlaveArmInteractionTimerCallback, this);
  } else if (arm_control_type == "normal_arm") {
    y1_interface_->SetArmControlMode(
        Y1SDKInterface::ControlMode::NRT_JOINT_POSITION);
    // subscriber
    // normal control arm receive control command.
    arm_end_pose_control_sub_ =
        nh_.subscribe(arm_end_pose_control_topic, 1,
                      &Y1Controller::ArmEndPoseControlCallback, this);
    arm_joint_position_control_sub_ =
        nh_.subscribe(arm_joint_position_control_topic, 1,
                      &Y1Controller::ArmJointPositionControlCallback, this);

  } else {
    ROS_ERROR("arm_control_type is %s , not supported",
              arm_control_type.c_str());
    return false;
  }

  // joint states publisher
  arm_joint_state_pub_ =
      nh_.advertise<y1_msg::ArmJointState>(arm_joint_state_topic, 1);
  // joint motor status publisher
  arm_status_pub_ = nh_.advertise<y1_msg::ArmStatus>(arm_status_topic, 1);

  // publish arm joint states at a fixed frequency
  arm_information_timer_ =
      nh_.createTimer(ros::Duration(1.0 / arm_feedback_rate),
                      &Y1Controller::ArmInformationTimerCallback, this);

  ROS_INFO("y1_controller success start!");

  return true;
}

void Y1Controller::ArmEndPoseControlCallback(
    const y1_msg::ArmEndPoseControl::ConstPtr& msg) {
  // end pose
  std::array<double, 6> end_pose;
  for (int i = 0; i < 6; i++) {
    end_pose[i] = msg->end_pose[i];
  }
  y1_interface_->SetArmEndPose(end_pose);
  // gripper stroke (mm)
  y1_interface_->SetGripperStroke(msg->gripper_stroke);
}

void Y1Controller::FollowArmJointPositionControlCallback(
    const y1_msg::ArmJointState::ConstPtr& msg) {
  if (msg->joint_position.size() >= 6) {
    // arm joint position
    y1_interface_->SetArmJointPosition(msg->joint_position);

    // 临时借改回调函数调试fdob用，传入主臂关节速度给从臂用
    y1_interface_->SetArmJointVelocity(msg->joint_velocity);

  } else {
    ROS_ERROR("follow arm receive joint control size < 6");
  }
}

void Y1Controller::ArmJointPositionControlCallback(
    const y1_msg::ArmJointPositionControl::ConstPtr& msg) {
  // joint position and velocity
  std::array<double, 6> joint_position;
  for (int i = 0; i < 6; i++) {
    joint_position[i] = msg->joint_position[i];
  }
  y1_interface_->SetArmJointPosition(joint_position, msg->joint_velocity);

  // gripper stroke (mm)
  y1_interface_->SetGripperStroke(msg->gripper_stroke, msg->gripper_velocity);
}

void Y1Controller::ArmInformationTimerCallback(const ros::TimerEvent&) {
  // robotic arm joint state
  y1_msg::ArmJointState arm_joint_state;
  arm_joint_state.header.stamp = ros::Time::now();

  // get arm end pose
  std::array<double, 6> arm_end_pose = y1_interface_->GetArmEndPose();

  // get 6 or 7(include gripper) joint position.
  std::vector<double> joint_position = y1_interface_->GetJointPosition();

  // get 6 or 7(include gripper) joint velocity.
  std::vector<double> joint_velocity = y1_interface_->GetJointVelocity();

  // get 6 or 7(include gripper) joint torque.
  std::vector<double> joint_effort = y1_interface_->GetJointEffort();

  for (int i = 0; i < joint_position.size(); i++) {
    arm_joint_state.joint_position.push_back(joint_position.at(i));
    arm_joint_state.joint_velocity.push_back(joint_velocity.at(i));
    arm_joint_state.joint_effort.push_back(joint_effort.at(i));
  }

  for (int i = 0; i < 6; i++) {
    arm_joint_state.end_pose.at(i) = arm_end_pose.at(i);
  }

  // joint motor status
  y1_msg::ArmStatus arm_status;
  arm_status.header.stamp = ros::Time::now();

  // get joint names
  std::vector<std::string> joint_names = y1_interface_->GetJointNames();

  // get motor current
  std::vector<double> motor_current = y1_interface_->GetMotorCurrent();

  // get rotor temperature
  std::vector<double> rotor_temperature = y1_interface_->GetRotorTemperature();

  // get joint motor error code
  std::vector<int> joint_error_code = y1_interface_->GetJointErrorCode();

  double total_current = 0;
  for (int i = 0; i < motor_current.size(); i++) {
    std_msgs::String joint_name;
    joint_name.data = joint_names.at(i);
    arm_status.name.push_back(joint_name);
    arm_status.motor_current.push_back(motor_current.at(i));
    arm_status.rotor_temperature.push_back(rotor_temperature.at(i));
    arm_status.error_code.push_back(joint_error_code.at(i));
    total_current += motor_current.at(i);
  }
  arm_status.motor_current.push_back(total_current);

  // publish arm information
  arm_joint_state_pub_.publish(arm_joint_state);
  arm_status_pub_.publish(arm_status);
}

void Y1Controller::SlaveArmInteractionTimerCallback(const ros::TimerEvent &) {
  y1_msg::InteractionForce interaction_force;

  interaction_force.header.stamp = ros::Time::now();

  interaction_force.arm_interaction_torque = y1_interface_->GetSlaveArmInteractionForce();

  interaction_force.grip_interaction_torque = y1_interface_->GetGripperInteractionForce();

  slave_arm_interaction_pub_.publish(interaction_force);
}

void Y1Controller::SlaveArmInteractionCallback(
    const y1_msg::InteractionForce::ConstPtr &msg) {

  y1_interface_->SetSlaveArmInteractionForce(msg->arm_interaction_torque);

  y1_interface_->SetSlaveGripperInteractionForce(msg->grip_interaction_torque);
}

}  // namespace y1_controller
}  // namespace imeta