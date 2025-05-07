#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <urdf/model.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "kdl_example");
    ros::NodeHandle nh;

    // 加载 URDF 模型
    std::string urdf_string;
    nh.param("robot_description", urdf_string, std::string());
    urdf::Model model;
    if (!model.initString(urdf_string)) {
        ROS_ERROR("Failed to parse URDF");
        return -1;
    }

    // 从 URDF 构建 KDL Tree
    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(model, kdl_tree)) {
        ROS_ERROR("Failed to construct KDL tree");
        return -1;
    }

    // 从 KDL Tree 提取 KDL Chain
    KDL::Chain kdl_chain;
    if (!kdl_tree.getChain("base_link", "tool0", kdl_chain)) {
        ROS_ERROR("Failed to get KDL chain");
        return -1;
    }

    // 初始化正向运动学求解器
    KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain);

    // 初始化逆向运动学求解器
    KDL::ChainIkSolverPos_LMA ik_solver(kdl_chain);

    // 设置关节角度
    KDL::JntArray joint_positions(kdl_chain.getNrOfJoints());
    for (unsigned int i = 0; i < joint_positions.rows(); ++i) {
        joint_positions(i) = 0.0; // 设置为零位
    }

    // 计算正向运动学
    KDL::Frame end_effector_pose;
    if (fk_solver.JntToCart(joint_positions, end_effector_pose) < 0) {
        ROS_ERROR("Failed to compute forward kinematics");
        return -1;
    }
    ROS_INFO_STREAM("End effector pose:\n" << end_effector_pose);

    // 设置目标位姿
    KDL::Frame desired_pose = end_effector_pose; // 这里使用当前位姿作为目标位姿

    // 计算逆向运动学
    KDL::JntArray result_joint_positions(kdl_chain.getNrOfJoints());
    if (ik_solver.CartToJnt(joint_positions, desired_pose, result_joint_positions) < 0) {
        ROS_ERROR("Failed to compute inverse kinematics");
        return -1;
    }
    ROS_INFO("Inverse kinematics result:");
    for (unsigned int i = 0; i < result_joint_positions.rows(); ++i) {
        ROS_INFO("Joint %d: %f", i, result_joint_positions(i));
    }

    return 0;
}
