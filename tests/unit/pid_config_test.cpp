#include "pid_config.hpp"

#include <gtest/gtest.h>

TEST(PidConfig, EachLoopNumberPointsAtItsGains)
{
  RobotParameters params;
  robot_pid::loopOf(params, 1)->set_k_p(1.f);
  robot_pid::loopOf(params, 2)->set_k_p(2.f);
  robot_pid::loopOf(params, 3)->set_k_p(3.f);
  robot_pid::loopOf(params, 4)->set_k_p(4.f);

  EXPECT_FLOAT_EQ(params.linear_pid().k_p(), 1.f);
  EXPECT_FLOAT_EQ(params.angular_pid().k_p(), 2.f);
  EXPECT_FLOAT_EQ(params.left_pid().k_p(), 3.f);
  EXPECT_FLOAT_EQ(params.right_pid().k_p(), 4.f);
  EXPECT_FLOAT_EQ(robot_pid::loopOf(params, robot_pid::Loop::LeftWheel).k_p(), 3.f);
}

TEST(PidConfig, AnUnknownLoopIsNull)
{
  RobotParameters params;
  EXPECT_EQ(robot_pid::loopOf(params, 0), nullptr);
  EXPECT_EQ(robot_pid::loopOf(params, 5), nullptr);
}

TEST(PidConfig, MergeKeepsWhatWasLeftBlank)
{
  PIDConfig current;
  current.set_k_p(1.f);
  current.set_k_i(0.5f);
  current.set_k_d(0.1f);
  current.set_enabled(true);

  const double k_p = 2.0;
  const bool enabled = false;
  robot_pid::PidUpdate update;
  update.k_p = &k_p;
  update.enabled = &enabled;

  const auto merged = robot_pid::merge(current, update);

  EXPECT_FLOAT_EQ(merged.k_p(), 2.f);
  EXPECT_FLOAT_EQ(merged.k_i(), 0.5f);
  EXPECT_FLOAT_EQ(merged.k_d(), 0.1f);
  EXPECT_FALSE(merged.enabled());
}
