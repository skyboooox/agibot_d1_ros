#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace agibot_d1_ros {

struct AxisLimit {
  const char *name;
  double official_min_abs;
  double official_max_abs;
  double safe_max_abs;
};

struct MoveCommand {
  double vx_mps{0.0};
  double vy_mps{0.0};
  double yaw_rate_radps{0.0};
};

struct CommandDecision {
  bool allowed{false};
  std::string reason{"blocked:not_evaluated"};
  MoveCommand command{};
  std::vector<std::string> notes{};
};

struct BridgePolicy {
  AxisLimit vx{"vx_mps", 0.05, 3.0, 0.15};
  AxisLimit vy{"vy_mps", 0.10, 1.0, 0.12};
  AxisLimit yaw{"yaw_rate_radps", 0.02, 3.0, 0.20};
  double publish_rate_hz{20.0};
  double watchdog_timeout_s{0.5};
  double stop_hold_s{0.5};
  double startup_stand_wait_s{4.0};
  double startup_stand_confirm_timeout_s{8.0};
  double startup_stand_confirm_poll_s{0.5};
  int min_battery_percent_for_live_motion{20};
};

inline int stop_frame_count(const BridgePolicy &policy) {
  return std::max(1, static_cast<int>(std::lround(policy.publish_rate_hz * policy.stop_hold_s)));
}

inline MoveCommand stop_command() {
  return MoveCommand{};
}

inline bool normalize_axis(const AxisLimit &limit, double input, double &output, std::string &note) {
  if (!std::isfinite(input)) {
    note = std::string("blocked:non_finite:") + limit.name;
    return false;
  }

  const double magnitude = std::abs(input);
  if (magnitude == 0.0) {
    output = 0.0;
    return true;
  }
  if (magnitude > limit.safe_max_abs) {
    note = std::string("blocked:velocity_limit:") + limit.name;
    return false;
  }
  if (magnitude < limit.official_min_abs) {
    output = 0.0;
    note = std::string("normalized:below_official_min:") + limit.name;
    return true;
  }

  output = input;
  return true;
}

inline CommandDecision normalize_twist(
    const BridgePolicy &policy,
    double linear_x,
    double linear_y,
    double angular_z) {
  CommandDecision decision;
  std::string note;

  if (!normalize_axis(policy.vx, linear_x, decision.command.vx_mps, note)) {
    decision.reason = note;
    return decision;
  }
  if (!note.empty()) {
    decision.notes.push_back(note);
    note.clear();
  }

  if (!normalize_axis(policy.vy, linear_y, decision.command.vy_mps, note)) {
    decision.reason = note;
    return decision;
  }
  if (!note.empty()) {
    decision.notes.push_back(note);
    note.clear();
  }

  if (!normalize_axis(policy.yaw, angular_z, decision.command.yaw_rate_radps, note)) {
    decision.reason = note;
    return decision;
  }
  if (!note.empty()) {
    decision.notes.push_back(note);
  }

  decision.allowed = true;
  decision.reason = "allowed";
  return decision;
}

}  // namespace agibot_d1_ros
