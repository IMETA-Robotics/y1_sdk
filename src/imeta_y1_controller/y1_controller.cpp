#include "y1_controller.h"

#include <memory>
#include <ros/package.h>

#include "imeta_y1_msg/ArmState.h"

namespace imeta {
namespace controller {
bool Y1Controller::Init() {
  // init ros parameters
  std::string can_id = nh_.param("arm_can_id", std::string("can1"));
  int arm_feedback_rate = nh_.param("arm_feedback_rate", 200);

  // end pose control mode
  std::string arm_end_pose_control_topic =
      nh_.param("arm_end_pose_control_topic",
                std::string("/y1_controller/arm_end_pose_control"));
  // joint position mode
  std::string arm_joint_position_control_topic =
      nh_.param("arm_joint_position_control_topic",
                std::string("/y1_controller/arm_joint_position_control_topic"));
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

  // init Y1 SDK Interface
  y1_interface_ =
      std::make_shared<Y1SDKInterface>(can_id, urdf_path, arm_end_type);
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
  arm_state_pub_ = nh_.advertise<imeta_y1_msg::ArmState>(arm_state_topic, 1);

  // publish arm joint states at a fixed frequency
  arm_state_timer_ =
      nh_.createTimer(ros::Duration(1.0 / arm_feedback_rate),
                      &Y1Controller::ArmStateTimerCallback, this);

  ROS_INFO("y1_controller success start!");

  return true;
}

void Y1Controller::ArmEndPoseControlCallback(
    const imeta_y1_msg::ArmEndPoseControl::ConstPtr& msg) {
  // end pose
  std::array<double, 6> arm_end_pose;
  for (int i = 0; i < 6; i++) {
    arm_end_pose[i] = msg->arm_end_pose[i];
  }
  y1_interface_->SetArmEndPose(arm_end_pose);

  // gripper joint position
  y1_interface_->SetGripperJointPosition(msg->gripper);
}

void Y1Controller::ArmJointPositionControlCallback(
    const imeta_y1_msg::ArmJointPositionControl::ConstPtr& msg) {
  // arm joint position
  std::array<double, 6> arm_joint_position;
  for (int i = 0; i < 6; i++) {
    arm_joint_position[i] = msg->arm_joint_position[i];
  }
  y1_interface_->SetArmJointPosition(arm_joint_position);

  // gripper joint position
  y1_interface_->SetGripperJointPosition(msg->gripper);
}

void Y1Controller::ArmStateTimerCallback(const ros::TimerEvent&) {
  imeta_y1_msg::ArmState arm_state;
  arm_state.header.stamp = ros::Time::now();

  // get arm end pose
  std::array<double, 6> arm_end_pose = y1_interface_->GetArmEndPose();

  // get 6 or 7(include gripper) joint position.
  std::vector<double> joint_position = y1_interface_->GetJointPosition();

  // get 6 or 7(include gripper) joint velocity.
  std::vector<double> joint_velocity = y1_interface_->GetJointVelocity();

  // get 6 or 7(include gripper) joint torque.
  std::vector<double> joint_effort = y1_interface_->GetJointEffort();

  for (int i = 0; i < joint_position.size(); i++) {
    arm_state.joint_position.push_back(joint_position.at(i));
    arm_state.joint_velocity.push_back(joint_velocity.at(i));
    arm_state.joint_effort.push_back(joint_effort.at(i));
  }

  for (int i = 0; i < 6; i++) {
    arm_state.end_pose.at(i) = arm_end_pose.at(i);
  }

  // publish arm state
  arm_state_pub_.publish(arm_state);
}

}  // namespace controller
}  // namespace imeta