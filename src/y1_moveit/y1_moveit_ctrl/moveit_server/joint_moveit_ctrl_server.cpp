#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <y1_moveit_ctrl/JointMoveitCtrl.h> // srv
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class JointMoveitCtrlServer {
public:
  JointMoveitCtrlServer(ros::NodeHandle &nh) {
    // 获取规划组列表
    robot_model_loader::RobotModelLoader robot_model_loader(
        "robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    const std::vector<std::string> &available_groups =
        kinematic_model->getJointModelGroupNames();

    ROS_INFO_STREAM("Available MoveIt groups: " << available_groups.size());

    // 初始化 MoveGroupInterface（如果存在）
    if (std::find(available_groups.begin(), available_groups.end(), "arm") !=
        available_groups.end()) {
      arm_group_ =
          std::make_shared<moveit::planning_interface::MoveGroupInterface>(
              "arm");
      ROS_INFO("Initialized arm move group.");
    }

    if (std::find(available_groups.begin(), available_groups.end(),
                  "gripper") != available_groups.end()) {
      gripper_group_ =
          std::make_shared<moveit::planning_interface::MoveGroupInterface>(
              "gripper");
      ROS_INFO("Initialized gripper move group.");
    }

    if (std::find(available_groups.begin(), available_groups.end(), "y1") !=
        available_groups.end()) {
      y1_group_ =
          std::make_shared<moveit::planning_interface::MoveGroupInterface>(
              "y1");
      ROS_INFO("Initialized imeta y1 move group.");
    }

    // ROS 服务
    arm_srv_ = nh.advertiseService("joint_moveit_ctrl_arm",
                                   &JointMoveitCtrlServer::handleArm, this);
    gripper_srv_ =
        nh.advertiseService("joint_moveit_ctrl_gripper",
                            &JointMoveitCtrlServer::handleGripper, this);
    y1_srv_ = nh.advertiseService("joint_moveit_ctrl_y1",
                                     &JointMoveitCtrlServer::handleY1, this);
    endpose_srv_ =
        nh.advertiseService("joint_moveit_ctrl_endpose",
                            &JointMoveitCtrlServer::handleEndpose, this);

    ROS_INFO("Joint MoveIt Control Services Ready.");
  }

private:
  // 处理 arm 关节运动
  bool handleArm(y1_moveit_ctrl::JointMoveitCtrl::Request &req,
                 y1_moveit_ctrl::JointMoveitCtrl::Response &res) {
    ROS_INFO("Received arm joint movement request.");
    if (!arm_group_) {
      ROS_ERROR("Arm move group is not initialized.");
      res.status = false;
      res.error_code = 1;
      return true;
    }

    try {
      std::vector<double> joint_goal(req.joint_states.begin(),
                                     req.joint_states.begin() + 6);
      arm_group_->setJointValueTarget(joint_goal);

      double max_velocity =
          std::max(1e-6, std::min(1.0 - 1e-6, req.max_velocity));
      double max_acceleration =
          std::max(1e-6, std::min(1.0 - 1e-6, req.max_acceleration));
      arm_group_->setMaxVelocityScalingFactor(max_velocity);
      arm_group_->setMaxAccelerationScalingFactor(max_acceleration);

      ROS_INFO("max_velocity: %f max_acceleration: %f", max_velocity,
               max_acceleration);
      arm_group_->move();
      res.status = true;
      res.error_code = 0;
    } catch (const std::exception &e) {
      ROS_ERROR("Exception during arm movement: %s", e.what());
      res.status = false;
      res.error_code = 2;
    }
    return true;
  }

  // 处理 gripper
  bool handleGripper(y1_moveit_ctrl::JointMoveitCtrl::Request &req,
                     y1_moveit_ctrl::JointMoveitCtrl::Response &res) {
    ROS_INFO("Received gripper joint movement request.");
    if (!gripper_group_) {
      ROS_ERROR("Gripper move group is not initialized.");
      res.status = false;
      res.error_code = 1;
      return true;
    }

    try {
      std::vector<double> gripper_goal = {req.gripper};
      gripper_group_->setJointValueTarget(gripper_goal);
      gripper_group_->move();
      res.status = true;
      res.error_code = 0;
    } catch (const std::exception &e) {
      ROS_ERROR("Exception during gripper movement: %s", e.what());
      res.status = false;
      res.error_code = 2;
    }
    return true;
  }

  // 处理 imeta_y1
  bool handleY1(y1_moveit_ctrl::JointMoveitCtrl::Request &req,
                   y1_moveit_ctrl::JointMoveitCtrl::Response &res) {
    ROS_INFO("Received imeta_y1 joint movement request.");
    if (!y1_group_) {
      ROS_ERROR("y1 move group is not initialized.");
      res.status = false;
      res.error_code = 1;
      return true;
    }

    try {
      std::vector<double> y1_goal(req.joint_states.begin(),
                                     req.joint_states.begin() + 6);
      y1_goal.push_back(req.gripper);

      y1_group_->setJointValueTarget(y1_goal);

      double max_velocity =
          std::max(1e-6, std::min(1.0 - 1e-6, req.max_velocity));
      double max_acceleration =
          std::max(1e-6, std::min(1.0 - 1e-6, req.max_acceleration));
      y1_group_->setMaxVelocityScalingFactor(max_velocity);
      y1_group_->setMaxAccelerationScalingFactor(max_acceleration);

      ROS_INFO("max_velocity: %f max_acceleration: %f", max_velocity,
               max_acceleration);
      y1_group_->move();
      res.status = true;
      res.error_code = 0;
    } catch (const std::exception &e) {
      ROS_ERROR("Exception during imeta_y1 movement: %s", e.what());
      res.status = false;
      res.error_code = 2;
    }
    return true;
  }

  // 处理 endpose
  bool handleEndpose(y1_moveit_ctrl::JointMoveitCtrl::Request &req,
                     y1_moveit_ctrl::JointMoveitCtrl::Response &res) {
    ROS_INFO("Received endpose movement request.");
    if (!arm_group_) {
      ROS_ERROR("Arm move group is not initialized.");
      res.status = false;
      res.error_code = 1;
      return true;
    }

    try {
      if (req.joint_endpose.size() != 7) {
        ROS_ERROR("Invalid joint_endpose size. Must be 7 (x,y,z,qx,qy,qz,qw).");
        res.status = false;
        res.error_code = 1;
        return true;
      }

      geometry_msgs::Pose target_pose;
      target_pose.position.x = req.joint_endpose[0];
      target_pose.position.y = req.joint_endpose[1];
      target_pose.position.z = req.joint_endpose[2];
      target_pose.orientation.x = req.joint_endpose[3];
      target_pose.orientation.y = req.joint_endpose[4];
      target_pose.orientation.z = req.joint_endpose[5];
      target_pose.orientation.w = req.joint_endpose[6];

      arm_group_->setPoseTarget(target_pose);

      double max_velocity =
          std::max(1e-6, std::min(1.0 - 1e-6, req.max_velocity));
      double max_acceleration =
          std::max(1e-6, std::min(1.0 - 1e-6, req.max_acceleration));
      arm_group_->setMaxVelocityScalingFactor(max_velocity);
      arm_group_->setMaxAccelerationScalingFactor(max_acceleration);

      ROS_INFO("max_velocity: %f max_acceleration: %f", max_velocity,
               max_acceleration);
      arm_group_->move();
      res.status = true;
      res.error_code = 0;
    } catch (const std::exception &e) {
      ROS_ERROR("Exception during endpose movement: %s", e.what());
      res.status = false;
      res.error_code = 2;
    }
    return true;
  }

private:
  ros::ServiceServer arm_srv_;
  ros::ServiceServer gripper_srv_;
  ros::ServiceServer y1_srv_;
  ros::ServiceServer endpose_srv_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface>
      gripper_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> y1_group_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "joint_moveit_ctrl_server");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  JointMoveitCtrlServer server(nh);

  ros::waitForShutdown();
  return 0;
}
