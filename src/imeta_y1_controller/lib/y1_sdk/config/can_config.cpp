#include "can_config.h"

namespace imeta {
namespace controller {

const std::vector<CanConfig> CanConfigs = {
{
    "can0", // std::string can_id;
        {
            // std::vector<MotorInfo>
            {   
                {
                    // MotorInfo 第一个电机
                    "J1", // joint_name
                    {0x11, "DmMotorReader"}, // motor_read_info
                    {0x01, "DmMotorWriter"}, // motor_write_info
                    -360, // position_min
                    360   // position_max
                },
                {
                    // MotorInfo 第二个电机
                    "J2", // joint_name
                    {0x12, "DmMotorReader"}, // motor_read_info
                    {0x02, "DmMotorWriter"}, // motor_write_info
                    -360, // position_min
                    360   // position_max
                },

            }
        }
    }
};

}
}