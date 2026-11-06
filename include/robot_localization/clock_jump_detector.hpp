// Copyright 2026 Kiwibot

#ifndef ROBOT_LOCALIZATION__CLOCK_JUMP_DETECTOR_HPP_
#define ROBOT_LOCALIZATION__CLOCK_JUMP_DETECTOR_HPP_

#include <cstdlib>
#include <optional>

#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"

namespace robot_localization
{

//! @brief Detects clock steps (forward or backward) between consecutive samples
//! that exceed a threshold. A non-positive threshold disables detection.
class ClockJumpDetector
{
public:
  explicit ClockJumpDetector(const rclcpp::Duration & threshold = rclcpp::Duration(0, 0))
  : threshold_(threshold) {}

  //! @return true if now differs from the previous sample by more than the threshold
  bool jumped(const rclcpp::Time & now)
  {
    const std::optional<rclcpp::Time> last = last_;
    last_ = now;
    if (!enabled() || !last) {
      return false;
    }
    return std::llabs((now - *last).nanoseconds()) > threshold_.nanoseconds();
  }

  bool enabled() const {return threshold_.nanoseconds() > 0;}

private:
  rclcpp::Duration threshold_;
  std::optional<rclcpp::Time> last_;
};

}  // namespace robot_localization

#endif  // ROBOT_LOCALIZATION__CLOCK_JUMP_DETECTOR_HPP_
