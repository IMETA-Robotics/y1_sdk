#pragma once

#include <vector>

namespace imeta {
namespace controller {

struct MotorState {
  double position;
  double velocity;
  double torque;
};

using MotorStateVector = std::vector<MotorState>;

}  // namespace controller
}  // namespace imeta