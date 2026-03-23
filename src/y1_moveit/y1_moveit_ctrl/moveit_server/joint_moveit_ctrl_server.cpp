#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
// 注意：ROS 2 的服务头文件路径通常是 pkg_name/srv/file_name.hpp
#include <y1_moveit_ctrl/srv/joint_moveit_ctrl.hpp> 
#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include <memory>
#include <algorithm>
#include <string>

class JointMoveitCtrlServer : public rclcpp::Node {
public:
  JointMoveitCtrlServer() : Node("joint_moveit_ctrl_server") {
    RCLCPP_INFO(this->get_logger(), "Starting JointMoveitCtrlServer...");

    // 在 ROS 2 中，RobotModelLoader 通常需要传入 node 和 param name
    // 但为了兼容性和简单性，我们直接尝试初始化 MoveGroupInterface
    // 如果组不存在，MoveGroupInterface 会抛出异常或返回 false (取决于版本)，这里用 try-catch 处理
    
    // 初始化 arm 组
    try {
      arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          shared_from_this(), "arm");
      RCLCPP_INFO(this->get_logger(), "Initialized arm move group.");
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "Failed to initialize arm group: %s", e.what());
      arm_group_ = nullptr;
    }

    // 初始化 gripper 组
    try {
      gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          shared_from_this(), "gripper");
      RCLCPP_INFO(this->get_logger(), "Initialized gripper move group.");
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "Failed to initialize gripper group: %s", e.what());
      gripper_group_ = nullptr;
    }

    // 初始化 y1 组
    try {
      y1_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          shared_from_this(), "y1");
      RCLCPP_INFO(this->get_logger(), "Initialized y1 move group.");
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "Failed to initialize y1 group: %s", e.what());
      y1_group_ = nullptr;
    }

    // 创建 ROS 2 服务
    // 注意：ROS 2 的回调函数签名不同，且通常使用 std::bind 或 lambda
    arm_srv_ = this->create_service<y1_moveit_ctrl::srv::JointMoveitCtrl>(
        "joint_moveit_ctrl_arm",
        std::bind(&JointMoveitCtrlServer::handleArm, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    gripper_srv_ = this->create_service<y1_moveit_ctrl::srv::JointMoveitCtrl>(
        "joint_moveit_ctrl_gripper",
        std::bind(&JointMoveitCtrlServer::handleGripper, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    y1_srv_ = this->create_service<y1_moveit_ctrl::srv::JointMoveitCtrl>(
        "joint_moveit_ctrl_y1",
        std::bind(&JointMoveitCtrlServer::handleY1, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    endpose_srv_ = this->create_service<y1_moveit_ctrl::srv::JointMoveitCtrl>(
        "joint_moveit_ctrl_endpose",
        std::bind(&JointMoveitCtrlServer::handleEndpose, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    RCLCPP_INFO(this->get_logger(), "Joint MoveIt Control Services Ready.");
  }

private:
  // ROS 2 服务回调签名：(request_header, request, response)
  // request_header 通常可以忽略
  void handleArm(const std::shared_ptr<rmw_request_id_t> request_header,
                 const std::shared_ptr<y1_moveit_ctrl::srv::JointMoveitCtrl::Request> req,
                 std::shared_ptr<y1_moveit_ctrl::srv::JointMoveitCtrl::Response> res) {
    (void)request_header; // 避免未使用变量警告
    RCLCPP_INFO(this->get_logger(), "Received arm joint movement request.");
    
    if (!arm_group_) {
      RCLCPP_ERROR(this->get_logger(), "Arm move group is not initialized.");
      res->status = false;
      res->error_code = 1;
      return;
    }

    try {
      std::vector<double> joint_goal(req->joint_states.begin(), req->joint_states.begin() + 6);
      arm_group_->setJointValueTarget(joint_goal);

      double max_velocity = std::max(1e-6, std::min(1.0 - 1e-6, req->max_velocity));
      double max_acceleration = std::max(1e-6, std::min(1.0 - 1e-6, req->max_acceleration));
      
      arm_group_->setMaxVelocityScalingFactor(max_velocity);
      arm_group_->setMaxAccelerationScalingFactor(max_acceleration);

      RCLCPP_INFO(this->get_logger(), "max_velocity: %f max_acceleration: %f", max_velocity, max_acceleration);
      
      moveit::core::MoveItErrorCode error_code = arm_group_->move();
      
      if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
        res->status = true;
        res->error_code = 0;
      } else {
        RCLCPP_ERROR(this->get_logger(), "Arm movement failed with code: %d", error_code.val);
        res->status = false;
        res->error_code = 2;
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception during arm movement: %s", e.what());
      res->status = false;
      res->error_code = 2;
    }
  }

  void handleGripper(const std::shared_ptr<rmw_request_id_t> request_header,
                     const std::shared_ptr<y1_moveit_ctrl::srv::JointMoveitCtrl::Request> req,
                     std::shared_ptr<y1_moveit_ctrl::srv::JointMoveitCtrl::Response> res) {
    (void)request_header;
    RCLCPP_INFO(this->get_logger(), "Received gripper joint movement request.");
    
    if (!gripper_group_) {
      RCLCPP_ERROR(this->get_logger(), "Gripper move group is not initialized.");
      res->status = false;
      res->error_code = 1;
      return;
    }

    try {
      std::vector<double> gripper_goal = {req->gripper};
      gripper_group_->setJointValueTarget(gripper_goal);
      
      moveit::core::MoveItErrorCode error_code = gripper_group_->move();
      
      if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
        res->status = true;
        res->error_code = 0;
      } else {
        res->status = false;
        res->error_code = 2;
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception during gripper movement: %s", e.what());
      res->status = false;
      res->error_code = 2;
    }
  }

  void handleY1(const std::shared_ptr<rmw_request_id_t> request_header,
                const std::shared_ptr<y1_moveit_ctrl::srv::JointMoveitCtrl::Request> req,
                std::shared_ptr<y1_moveit_ctrl::srv::JointMoveitCtrl::Response> res) {
    (void)request_header;
    RCLCPP_INFO(this->get_logger(), "Received imeta_y1 joint movement request.");
    
    if (!y1_group_) {
      RCLCPP_ERROR(this->get_logger(), "y1 move group is not initialized.");
      res->status = false;
      res->error_code = 1;
      return;
    }

    try {
      std::vector<double> y1_goal(req->joint_states.begin(), req->joint_states.begin() + 6);
      y1_goal.push_back(req->gripper);

      y1_group_->setJointValueTarget(y1_goal);

      double max_velocity = std::max(1e-6, std::min(1.0 - 1e-6, req->max_velocity));
      double max_acceleration = std::max(1e-6, std::min(1.0 - 1e-6, req->max_acceleration));
      
      y1_group_->setMaxVelocityScalingFactor(max_velocity);
      y1_group_->setMaxAccelerationScalingFactor(max_acceleration);

      RCLCPP_INFO(this->get_logger(), "max_velocity: %f max_acceleration: %f", max_velocity, max_acceleration);
      
      moveit::core::MoveItErrorCode error_code = y1_group_->move();
      
      if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
        res->status = true;
        res->error_code = 0;
      } else {
        res->status = false;
        res->error_code = 2;
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception during imeta_y1 movement: %s", e.what());
      res->status = false;
      res->error_code = 2;
    }
  }

  void handleEndpose(const std::shared_ptr<rmw_request_id_t> request_header,
                     const std::shared_ptr<y1_moveit_ctrl::srv::JointMoveitCtrl::Request> req,
                     std::shared_ptr<y1_moveit_ctrl::srv::JointMoveitCtrl::Response> res) {
    (void)request_header;
    RCLCPP_INFO(this->get_logger(), "Received endpose movement request.");
    
    if (!arm_group_) {
      RCLCPP_ERROR(this->get_logger(), "Arm move group is not initialized.");
      res->status = false;
      res->error_code = 1;
      return;
    }

    try {
      if (req->joint_endpose.size() != 7) {
        RCLCPP_ERROR(this->get_logger(), "Invalid joint_endpose size. Must be 7 (x,y,z,qx,qy,qz,qw).");
        res->status = false;
        res->error_code = 1;
        return;
      }

      geometry_msgs::msg::Pose target_pose;
      target_pose.position.x = req->joint_endpose[0];
      target_pose.position.y = req->joint_endpose[1];
      target_pose.position.z = req->joint_endpose[2];
      target_pose.orientation.x = req->joint_endpose[3];
      target_pose.orientation.y = req->joint_endpose[4];
      target_pose.orientation.z = req->joint_endpose[5];
      target_pose.orientation.w = req->joint_endpose[6];

      arm_group_->setPoseTarget(target_pose);

      double max_velocity = std::max(1e-6, std::min(1.0 - 1e-6, req->max_velocity));
      double max_acceleration = std::max(1e-6, std::min(1.0 - 1e-6, req->max_acceleration));
      
      arm_group_->setMaxVelocityScalingFactor(max_velocity);
      arm_group_->setMaxAccelerationScalingFactor(max_acceleration);

      RCLCPP_INFO(this->get_logger(), "max_velocity: %f max_acceleration: %f", max_velocity, max_acceleration);
      
      moveit::core::MoveItErrorCode error_code = arm_group_->move();
      
      if (error_code == moveit::core::MoveItErrorCode::SUCCESS) {
        res->status = true;
        res->error_code = 0;
      } else {
        res->status = false;
        res->error_code = 2;
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception during endpose movement: %s", e.what());
      res->status = false;
      res->error_code = 2;
    }
  }

private:
  // ROS 2 Service Server 指针
  rclcpp::Service<y1_moveit_ctrl::srv::JointMoveitCtrl>::SharedPtr arm_srv_;
  rclcpp::Service<y1_moveit_ctrl::srv::JointMoveitCtrl>::SharedPtr gripper_srv_;
  rclcpp::Service<y1_moveit_ctrl::srv::JointMoveitCtrl>::SharedPtr y1_srv_;
  rclcpp::Service<y1_moveit_ctrl::srv::JointMoveitCtrl>::SharedPtr endpose_srv_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> y1_group_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  
  // 创建节点
  auto server_node = std::make_shared<JointMoveitCtrlServer>();
  
  // 使用 MultiThreadedExecutor 以更好地处理并发服务请求和 MoveIt 后台线程
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(server_node);
  
  RCLCPP_INFO(server_node->get_logger(), "Spinning JointMoveitCtrlServer...");
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}