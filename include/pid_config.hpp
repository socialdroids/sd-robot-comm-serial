#pragma once

#include "feedback.pb.h"

namespace robot_pid
{

// The base's control loops, numbered as sd_msgs/srv/SetBasePid.
enum class Loop : uint8_t { Linear = 1, Angular = 2, LeftWheel = 3, RightWheel = 4 };

// The loop's gains inside `params`, or nullptr for an unknown loop.
PIDConfig * loopOf(RobotParameters & params, uint8_t loop);
const PIDConfig & loopOf(const RobotParameters & params, Loop loop);

// What came to replace a loop's gains; a null field keeps the current value
// (the web page sends null for a field left blank). The package builds as
// C++14, hence pointers and not std::optional.
struct PidUpdate
{
  const double * k_p = nullptr;
  const double * k_i = nullptr;
  const double * k_d = nullptr;
  const bool * enabled = nullptr;
};

PIDConfig merge(const PIDConfig & current, const PidUpdate & update);

}  // namespace robot_pid
