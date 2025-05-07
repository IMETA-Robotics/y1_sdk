#pragma once

#include <ros/ros.h>

#include <memory>
#include <string>

#include "imeta_y1_msg/ArmControl.h"
#include "imeta_y1_msg/ArmState.h"
#include "ros/subscriber.h"

namespace imeta {
namespace controller {

/**
 * @class Y1Controller
 * @brief ros1 interface for y1 controller. (ros1 noetic for ubuntu 20.04)
 */
class Y1Controller {
 public:
  Y1Controller() : nh_("~") {}
  ~Y1Controller() = default;

 public:
  /**
   * @brief init all related class of y1 controller.
   * @result must be true to run normally.
   */
  bool Init();

 private:
  /**
   * @brief init all related class of y1 controller.
   * @result must be true to run normally.
   */
  bool InitParams();

  /**
   * @brief init all publishers and subscribers.
   */
  void InitTopic();

  /**
   * @brief receive arm control command. support end pose or joint position
   * control mode.
   */
  void ArmControlCallback(const imeta_y1_msg::ArmControl::ConstPtr& msg);

  /**
   * @brief publish arm joint states at a fixed frequency
   */
  void ArmStateTimerCallback(const ros::TimerEvent&);

 private:
  ros::NodeHandle nh_;
  ros::Publisher arm_state_pub_;
  ros::Subscriber arm_control_sub_;

  ros::Timer arm_state_timer_;
};

}  // namespace controller
}  // namespace imeta
