#include "motor_config.h"

namespace imeta {
namespace controller {

const std::vector<MotorInfo> kMotorInfos = {
    {
        // MotorInfo 第一个电机
        "J1",                     // joint_name
        {0x11, "DmMotorReader"},  // motor_read_info
        {0x01, "DmMotorWriter"},  // motor_write_info
        12.5,                     // pmax
        30,                       // vmax
        10,                       // tmax
        -360,                     // position_min
        360                       // position_max
    },
    {
        // MotorInfo 第二个电机
        "J2",                     // joint_name
        {0x12, "DmMotorReader"},  // motor_read_info
        {0x02, "DmMotorWriter"},  // motor_write_info
        12.5,                     // pmax
        30,                       // vmax
        10,                       // tmax
        -360,                     // position_min
        360                       // position_max
    },
    {
        // MotorInfo 第三个电机
        "J3",                     // joint_name
        {0x13, "DmMotorReader"},  // motor_read_info
        {0x03, "DmMotorWriter"},  // motor_write_info
        12.5,                     // pmax
        30,                       // vmax
        10,                       // tmax
        -360,                     // position_min
        360                       // position_max
    },
    {
        // MotorInfo 第四个电机
        "J4",                     // joint_name
        {0x14, "DmMotorReader"},  // motor_read_info
        {0x04, "DmMotorWriter"},  // motor_write_info
        12.5,                     // pmax
        30,                       // vmax
        10,                       // tmax
        -360,                     // position_min
        360                       // position_max
    },
    {
        // MotorInfo 第五个电机
        "J5",                     // joint_name
        {0x15, "DmMotorReader"},  // motor_read_info
        {0x05, "DmMotorWriter"},  // motor_write_info
        12.5,                     // pmax
        30,                       // vmax
        10,                       // tmax
        -360,                     // position_min
        360                       // position_max
    },
    {
        // MotorInfo 第六个电机
        "J6",                     // joint_name
        {0x16, "DmMotorReader"},  // motor_read_info
        {0x06, "DmMotorWriter"},  // motor_write_info
        12.5,                     // pmax
        30,                       // vmax
        10,                       // tmax
        -360,                     // position_min
        360                       // position_max
    },
    {
        // MotorInfo 末端夹爪或示教器
        "Gripper",                // joint_name
        {0x17, "DmMotorReader"},  // motor_read_info
        {0x07, "DmMotorWriter"},  // motor_write_info
        12.5,                     // pmax
        30,                       // vmax
        10,                       // tmax
        -360,                     // position_min
        360                       // position_max
    }

};

}  // namespace controller
}  // namespace imeta