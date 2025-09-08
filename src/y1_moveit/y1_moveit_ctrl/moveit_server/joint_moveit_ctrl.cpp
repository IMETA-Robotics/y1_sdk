#include <ros/ros.h>
#include <y1_moveit_ctrl/JointMoveitCtrl.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cstdlib>
#include <ctime>
#include <vector>
#include <thread>
#include <chrono>

bool call_joint_moveit_ctrl_arm(ros::NodeHandle& nh,
                                const std::vector<double>& joint_states,
                                double max_velocity = 0.5,
                                double max_acceleration = 0.5)
{
    ros::ServiceClient client = nh.serviceClient<y1_moveit_ctrl::JointMoveitCtrl>("joint_moveit_ctrl_arm");
    client.waitForExistence();

    y1_moveit_ctrl::JointMoveitCtrl srv;
    // srv.request.joint_states = joint_states;
    if (joint_states.size() == 6) {
      std::copy(joint_states.begin(), joint_states.end(),
                srv.request.joint_states.begin());
    } else {
      ROS_ERROR("joint_states size is not 6. Cannot assign to "
                "std::array<double, 6>.");
    }
    srv.request.gripper = 0.0;
    srv.request.max_velocity = max_velocity;
    srv.request.max_acceleration = max_acceleration;

    if (client.call(srv))
    {
        if (srv.response.status)
            ROS_INFO("Successfully executed joint_moveit_ctrl_arm");
        else
            ROS_WARN("Failed to execute joint_moveit_ctrl_arm, error code: %d", srv.response.error_code);
        return srv.response.status;
    }
    else
    {
        ROS_ERROR("Service call failed: joint_moveit_ctrl_arm");
        return false;
    }
}

bool call_joint_moveit_ctrl_gripper(ros::NodeHandle& nh,
                                    double gripper_position,
                                    double max_velocity = 0.5,
                                    double max_acceleration = 0.5)
{
    ros::ServiceClient client = nh.serviceClient<y1_moveit_ctrl::JointMoveitCtrl>("joint_moveit_ctrl_gripper");
    client.waitForExistence();

    y1_moveit_ctrl::JointMoveitCtrl srv;
    // srv.request.joint_states = std::vector<double>(6, 0.0);
    srv.request.joint_states = {0,0,0,0,0, 0};
    srv.request.gripper = gripper_position;
    srv.request.max_velocity = max_velocity;
    srv.request.max_acceleration = max_acceleration;

    if (client.call(srv))
    {
        if (srv.response.status)
            ROS_INFO("Successfully executed joint_moveit_ctrl_gripper");
        else
            ROS_WARN("Failed to execute joint_moveit_ctrl_gripper, error code: %d", srv.response.error_code);
        return srv.response.status;
    }
    else
    {
        ROS_ERROR("Service call failed: joint_moveit_ctrl_gripper");
        return false;
    }
}

bool call_joint_moveit_ctrl_y1(ros::NodeHandle& nh,
                                  const std::vector<double>& joint_states,
                                  double gripper_position,
                                  double max_velocity = 0.5,
                                  double max_acceleration = 0.5)
{
    ros::ServiceClient client = nh.serviceClient<y1_moveit_ctrl::JointMoveitCtrl>("joint_moveit_ctrl_y1");
    client.waitForExistence();

    y1_moveit_ctrl::JointMoveitCtrl srv;
    // srv.request.joint_states = joint_states;
    if (joint_states.size() == 6) {
      std::copy(joint_states.begin(), joint_states.end(),
                srv.request.joint_states.begin());
    } else {
      ROS_ERROR("joint_states size is not 6. Cannot assign to "
                "std::array<double, 6>.");
    }
    srv.request.gripper = gripper_position;
    srv.request.max_velocity = max_velocity;
    srv.request.max_acceleration = max_acceleration;

    if (client.call(srv))
    {
        if (srv.response.status)
            ROS_INFO("Successfully executed joint_moveit_ctrl_y1");
        else
            ROS_WARN("Failed to execute joint_moveit_ctrl_y1, error code: %d", srv.response.error_code);
        return srv.response.status;
    }
    else
    {
        ROS_ERROR("Service call failed: joint_moveit_ctrl_y1");
        return false;
    }
}

std::vector<double> convert_endpose(const std::vector<double>& endpose)
{
    if (endpose.size() == 6)
    {
        double x = endpose[0], y = endpose[1], z = endpose[2];
        double roll = endpose[3], pitch = endpose[4], yaw = endpose[5];

        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);

        return {x, y, z, q.x(), q.y(), q.z(), q.w()};
    }
    else if (endpose.size() == 7)
    {
        return endpose;
    }
    else
    {
        throw std::runtime_error("Invalid endpose format! Must be 6 (Euler) or 7 (Quaternion) values.");
    }
}

bool call_joint_moveit_ctrl_endpose(ros::NodeHandle& nh,
                                    const std::vector<double>& endpose,
                                    double max_velocity = 0.5,
                                    double max_acceleration = 0.5)
{
    ros::ServiceClient client = nh.serviceClient<y1_moveit_ctrl::JointMoveitCtrl>("joint_moveit_ctrl_endpose");
    client.waitForExistence();

    y1_moveit_ctrl::JointMoveitCtrl srv;
    // srv.request.joint_states = std::vector<double>(6, 0.0);
    srv.request.joint_states = {0,0,0,0,0,0};
    srv.request.gripper = 0.0;
    srv.request.max_velocity = max_velocity;
    srv.request.max_acceleration = max_acceleration;
    // TODO:需要适配
    auto endpose_convert = convert_endpose(endpose);
    // srv.request.joint_endpose = convert_endpose(endpose);
    std::copy(endpose_convert.begin(), endpose_convert.end(),
              srv.request.joint_endpose.begin());

    if (client.call(srv))
    {
        if (srv.response.status)
            ROS_INFO("Successfully executed joint_moveit_ctrl_endpose");
        else
            ROS_WARN("Failed to execute joint_moveit_ctrl_endpose, error code: %d", srv.response.error_code);
        return srv.response.status;
    }
    else
    {
        ROS_ERROR("Service call failed: joint_moveit_ctrl_endpose");
        return false;
    }
}

std::pair<std::vector<double>, double> randomval()
{
    std::vector<double> arm_position = {
        ((double)rand() / RAND_MAX) * 0.4 - 0.2,
        ((double)rand() / RAND_MAX) * 0.5,
        ((double)rand() / RAND_MAX) * -0.5,
        ((double)rand() / RAND_MAX) * 0.4 - 0.2,
        ((double)rand() / RAND_MAX) * 0.4 - 0.2,
        ((double)rand() / RAND_MAX) * 0.4 - 0.2};

    double gripper_position = ((double)rand() / RAND_MAX) * 0.035;

    return {arm_position, gripper_position};
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_joint_moveit_ctrl");
    ros::NodeHandle nh;
    srand(time(nullptr));

    for (int i = 0; i < 10; i++)
    {
        // 测试不同模式，和 Python 脚本一致，可以根据需要取消注释

        // // Arm 控制
        // auto [arm_position, _] = randomval();
        // call_joint_moveit_ctrl_arm(nh, arm_position);
        // std::this_thread::sleep_for(std::chrono::seconds(1));

        // // Gripper 控制
        // auto [_, gripper_position] = randomval();
        // call_joint_moveit_ctrl_gripper(nh, gripper_position);
        // std::this_thread::sleep_for(std::chrono::seconds(1));

        // // y1 控制
        // auto [arm_position2, gripper_position2] = randomval();
        // call_joint_moveit_ctrl_y1(nh, arm_position2, gripper_position2);
        // std::this_thread::sleep_for(std::chrono::seconds(1));

        // // Endpose 欧拉角
        // std::vector<double> endpose_euler = {0.531014, -0.133376, 0.418909,
        //                                     -0.6052452780065936, 1.2265301318390152, -0.9107128036906411};
        // call_joint_moveit_ctrl_endpose(nh, endpose_euler);
        // std::this_thread::sleep_for(std::chrono::seconds(1));

        // // Endpose 四元数
        // std::vector<double> endpose_quat = {0.531014, -0.133376, 0.418909,
        //                                     0.02272779901175584, 0.6005891177332143,
        //                                     -0.18925185045722595, 0.7765049233012219};
        // call_joint_moveit_ctrl_endpose(nh, endpose_quat);
        // std::this_thread::sleep_for(std::chrono::seconds(1));

        // 回零
        std::vector<double> arm_zero = {0, 0, 1, 0, 0, 0};
        call_joint_moveit_ctrl_arm(nh, arm_zero);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}
