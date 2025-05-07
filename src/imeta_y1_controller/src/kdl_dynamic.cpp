#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <urdf/model.h>
#include <kdl/jntarray.hpp>
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

// 初始化KDL求解器
KDL::ChainIdSolver_RNE id_solver(kdl_chain, KDL::Vector(0, 0, -9.81));

// 关节数量
const size_t num_joints = kdl_chain.getNrOfJoints();

// 初始化关节状态
KDL::JntArray joint_positions(num_joints);
KDL::JntArray joint_velocities(num_joints);
KDL::JntArray joint_accelerations(num_joints);
KDL::JntArray joint_torques(num_joints);

// 初始化卡尔曼滤波器数组
std::vector<KalmanFilter> kalman_filters;
for (size_t i = 0; i < num_joints; ++i) {
    kalman_filters.emplace_back(0.01, 1e-5, 1e-2); // dt, process_noise, measurement_noise
}

// 在控制循环中更新关节状态
for (size_t i = 0; i < num_joints; ++i) {
    double measured_velocity = getMeasuredVelocity(i); // 获取当前关节速度
    double control_input = getControlInput(i); // 获取控制输入，加速度的估计值或其他

    kalman_filters[i].predict(control_input);
    kalman_filters[i].update(measured_velocity);

    joint_positions(i) = getMeasuredPosition(i); // 获取当前关节位置
    joint_velocities(i) = kalman_filters[i].getVelocity();
    joint_accelerations(i) = control_input; // 或者使用其他方法估算加速度
}

// 计算动力学补偿力矩
id_solver.CartToJnt(joint_positions, joint_velocities, joint_accelerations, joint_torques);

// 将计算得到的力矩作为补偿项应用于控制器 (力矩控制?)
applyTorqueCompensation(joint_torques);
