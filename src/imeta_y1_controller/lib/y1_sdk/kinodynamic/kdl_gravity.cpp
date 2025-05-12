#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chaindynparam.hpp>
#include <urdf/model.h>
#include <vector>
#include <iostream>

// 假设urdf_file是URDF文件的路径
std::string urdf_file = "path_to_urdf_file.urdf";

// 加载URDF模型
urdf::Model model;
if (!model.initFile(urdf_file)) {
    std::cerr << "Failed to parse urdf file" << std::endl;
    return -1;
}

// 将URDF模型转换为KDL树
KDL::Tree kdl_tree;
if (!kdl_parser::treeFromUrdfModel(model, kdl_tree)) {
    std::cerr << "Failed to construct kdl tree" << std::endl;
    return -1;
}

// 定义基座和末端执行器的链接名称
std::string base_link = "base_link";
std::string end_effector_link = "end_effector_link";

// 从KDL树中提取KDL链
KDL::Chain kdl_chain;
if (!kdl_tree.getChain(base_link, end_effector_link, kdl_chain)) {
    std::cerr << "Failed to get KDL chain" << std::endl;
    return -1;
}

// 定义重力向量（例如，沿Z轴向下）
KDL::Vector gravity(0.0, 0.0, -9.81);

// 初始化动力学参数计算器
KDL::ChainDynParam dyn_param(kdl_chain, gravity);

// 获取关节数
unsigned int num_joints = kdl_chain.getNrOfJoints();

// 定义关节位置和重力补偿力矩
KDL::JntArray joint_positions(num_joints);
KDL::JntArray gravity_torques(num_joints);

// 假设joint_positions已被填充为当前关节位置
// 计算重力补偿力矩
dyn_param.JntToGravity(joint_positions, gravity_torques);

// 输出每个关节的重力补偿力矩
for (unsigned int i = 0; i < num_joints; i++) {
    std::cout << "Joint " << i << ": " << gravity_torques(i) << " Nm" << std::endl;
}
