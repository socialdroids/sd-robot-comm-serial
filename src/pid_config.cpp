#include "pid_config.hpp"

namespace robot_pid
{

PIDConfig * loopOf(RobotParameters & params, uint8_t loop)
{
  switch (static_cast<Loop>(loop)) {
    case Loop::Linear:
      return params.mutable_linear_pid();
    case Loop::Angular:
      return params.mutable_angular_pid();
    case Loop::LeftWheel:
      return params.mutable_left_pid();
    case Loop::RightWheel:
      return params.mutable_right_pid();
  }
  return nullptr;
}

const PIDConfig & loopOf(const RobotParameters & params, Loop loop)
{
  switch (loop) {
    case Loop::Linear:
      return params.linear_pid();
    case Loop::Angular:
      return params.angular_pid();
    case Loop::LeftWheel:
      return params.left_pid();
    case Loop::RightWheel:
      break;
  }
  return params.right_pid();
}

PIDConfig merge(const PIDConfig & current, const PidUpdate & update)
{
  PIDConfig merged = current;
  if (update.k_p) {merged.set_k_p(static_cast<float>(*update.k_p));}
  if (update.k_i) {merged.set_k_i(static_cast<float>(*update.k_i));}
  if (update.k_d) {merged.set_k_d(static_cast<float>(*update.k_d));}
  if (update.enabled) {merged.set_enabled(*update.enabled);}
  return merged;
}

}  // namespace robot_pid
