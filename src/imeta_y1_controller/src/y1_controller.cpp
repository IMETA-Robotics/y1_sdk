#include "y1_controller.h"

#include <ros/package.h>

#include "imeta_y1_msg/ArmControl.h"
#include "imeta_y1_msg/ArmState.h"

namespace imeta {
namespace controller {
bool Y1Controller::Init() {
  // init ros parameters
  // InitParams();
  std::string can_id = nh_.param("arm_can_id", std::string("can1"));
  int arm_feedback_rate = nh_.param("arm_feedback_rate", 200);
  std::string arm_control_topic =
      nh_.param("arm_control_topic", std::string("/y1_controller/arm_control"));
  std::string arm_state_topic =
      nh_.param("arm_feedback_topic", std::string("/y1_controller/arm_state"));

  // get urdf path
  // 暂时用松灵Piper的urdf测试
  std::string urdf_path = ros::package::getPath("imeta_y1_controller") +
                          "/urdf/piper_description.urdf";
  // TODO: init Y1 SDK Interface
  

  // publisher
  arm_state_pub_ = nh_.advertise<imeta_y1_msg::ArmState>(arm_state_topic, 1);

  // subscriber
  arm_control_sub_ = nh_.subscribe(arm_control_topic, 1,
                                   &Y1Controller::ArmControlCallback, this);
  // InitTopic();

  // publish arm joint states at a fixed frequency
  arm_state_timer_ =
      nh_.createTimer(ros::Duration(1.0 / arm_feedback_rate),
                      &Y1Controller::ArmStateTimerCallback, this);

  ROS_INFO("y1_controller success start!");

  return true;
}

void Y1Controller::ArmControlCallback(
    const imeta_y1_msg::ArmControl::ConstPtr& msg) {
  // TODO: end pose or joint position control arm
}

void Y1Controller::ArmStateTimerCallback(const ros::TimerEvent&) {
  imeta_y1_msg::ArmState arm_state;
  // TODO: publish arm state
}

// bool Y1Controller::InitParams() {

// }

}  // namespace controller
}  // namespace imeta