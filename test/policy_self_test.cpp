#include <cmath>
#include <iostream>
#include <string>

#include "agibot_d1_ros/highlevel_policy.hpp"

namespace {

int failures = 0;

void require(bool condition, const std::string &message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++failures;
  }
}

void require_close(double actual, double expected, const std::string &message) {
  require(std::abs(actual - expected) < 1e-9, message);
}

}  // namespace

int main() {
  const agibot_d1_ros::BridgePolicy policy;

  const auto stop = agibot_d1_ros::normalize_twist(policy, 0.0, 0.0, 0.0);
  require(stop.allowed, "zero stop must be allowed");
  require_close(stop.command.vx_mps, 0.0, "zero vx");
  require_close(stop.command.vy_mps, 0.0, "zero vy");
  require_close(stop.command.yaw_rate_radps, 0.0, "zero yaw");

  const auto forward = agibot_d1_ros::normalize_twist(policy, 0.1, 0.0, 0.0);
  require(forward.allowed, "bounded forward must be allowed");
  require_close(forward.command.vx_mps, 0.1, "bounded forward vx");

  const auto below_min = agibot_d1_ros::normalize_twist(policy, 0.02, 0.0, 0.0);
  require(below_min.allowed, "below official min must normalize to stop");
  require_close(below_min.command.vx_mps, 0.0, "below official min vx");
  require(!below_min.notes.empty(), "below official min must report note");

  const auto over_limit = agibot_d1_ros::normalize_twist(policy, 0.3, 0.0, 0.0);
  require(!over_limit.allowed, "over safe limit must block");
  require(over_limit.reason == "blocked:velocity_limit:vx_mps", "over limit reason");

  const auto non_finite = agibot_d1_ros::normalize_twist(policy, NAN, 0.0, 0.0);
  require(!non_finite.allowed, "NaN must block");
  require(non_finite.reason == "blocked:non_finite:vx_mps", "NaN reason");

  require(agibot_d1_ros::stop_frame_count(policy) == 10, "stop frame count at 20 Hz for 0.5 s");

  if (failures != 0) {
    return 1;
  }
  std::cout << "policy_self_test ok\n";
  return 0;
}
