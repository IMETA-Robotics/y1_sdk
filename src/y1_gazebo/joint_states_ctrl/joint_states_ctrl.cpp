#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

#include <map>
#include <string>
#include <vector>
#include <cmath>

// 关节列表
std::vector<std::string> joint_names = {
    "joint1", "joint2", "joint3", "joint4",
    "joint5", "joint6", "joint7"
};

// Publisher 列表
std::map<std::string, ros::Publisher> publishers;

// 缓存上一次的关节位置
std::map<std::string, double> last_positions;

// 回调函数
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    std::map<std::string, double> joint_positions;

    // std::cout << "接受到 joint_states 消息" << std::endl;

    // 将 JointState 转换为 map
    for (size_t i = 0; i < msg->name.size(); ++i) {
    // std::cout << "关节: " << msg->name[i] << " 位置: " << msg->position[i] << std::endl;
        joint_positions[msg->name[i]] = msg->position[i];
    }

    for (const auto& joint_name : joint_names) {
        double position = 0.0;

        // if (joint_name == "joint8") {
        //     // joint8 = -joint7
        //     if (joint_positions.find("joint7") != joint_positions.end()) {
        //         position = -joint_positions["joint7"];
        //         // position = joint_positions["joint7"];
        //     } else {
        //         position = 0.0;
        //     }
        // } else {
            // 其他关节
            if (joint_positions.find(joint_name) != joint_positions.end()) {
                position = joint_positions[joint_name];
            } else {
                continue;  // 没有该关节数据
            }
        // }

        // 仅在关节位置发生变化时发布
        if (last_positions.find(joint_name) == last_positions.end() ||
            std::fabs(position - last_positions[joint_name]) > 1e-5) 
        {
            std_msgs::Float64 cmd;
            cmd.data = position;
            publishers[joint_name].publish(cmd);
            std::cout << "发布 " << joint_name << " 位置: " << position << std::endl;
            last_positions[joint_name] = position;
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joint_states_ctrl");
    ros::NodeHandle nh;

    // 初始化 publishers
    for (const auto& name : joint_names) {
        std::string topic = "/gazebo/" + name + "_position_controller/command";
        publishers[name] = nh.advertise<std_msgs::Float64>(topic, 10);
    }

    // 订阅 joint_states
    ros::Subscriber sub = nh.subscribe("/joint_states", 1, jointStateCallback);

    ros::spin();
    return 0;
}
