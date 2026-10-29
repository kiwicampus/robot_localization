// Copyright 2026 Kiwibot

#include "gtest/gtest.h"
#include "robot_localization/clock_jump_detector.hpp"

using robot_localization::ClockJumpDetector;

namespace
{
rclcpp::Time seconds(double s) {return rclcpp::Time(static_cast<int64_t>(s * 1e9));}
}  // namespace

TEST(ClockJumpDetector, DisabledNeverJumps) {
  ClockJumpDetector detector;
  EXPECT_FALSE(detector.enabled());
  EXPECT_FALSE(detector.jumped(seconds(1.0)));
  EXPECT_FALSE(detector.jumped(seconds(100000.0)));
}

TEST(ClockJumpDetector, FirstSampleIsNotAJump) {
  ClockJumpDetector detector(rclcpp::Duration::from_seconds(10.0));
  EXPECT_FALSE(detector.jumped(seconds(1793228639.0)));
}

TEST(ClockJumpDetector, DetectsForwardAndBackwardSteps) {
  ClockJumpDetector detector(rclcpp::Duration::from_seconds(10.0));
  detector.jumped(seconds(100.0));
  EXPECT_FALSE(detector.jumped(seconds(100.02)));
  EXPECT_TRUE(detector.jumped(seconds(55565.0)));
  EXPECT_FALSE(detector.jumped(seconds(55565.02)));
  EXPECT_TRUE(detector.jumped(seconds(200.0)));
  EXPECT_FALSE(detector.jumped(seconds(209.0)));
}
