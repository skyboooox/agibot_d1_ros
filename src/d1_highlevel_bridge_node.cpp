#include <chrono>
#include <algorithm>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>

#include "agibot_d1_ros/highlevel_policy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "zsl-1/highlevel.h"

using namespace std::chrono_literals;

namespace agibot_d1_ros {

constexpr uint32_t kDampingCtrlMode = 0;
constexpr uint32_t kStandingCtrlMode = 1;
constexpr uint32_t kMovingCtrlMode = 18;
constexpr double kMinimumStartupStandWaitS = 5.0;

bool is_standing_ctrl_mode(uint32_t ctrl_mode) {
  return ctrl_mode == kStandingCtrlMode;
}

class D1HighlevelBridgeNode final : public rclcpp::Node {
 public:
  D1HighlevelBridgeNode() : Node("agibot_d1_highlevel_bridge") {
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/agibot_d1/cmd_vel_safe");
    status_topic_ = declare_parameter<std::string>("status_topic", "/agibot_d1/bridge_status");
    dry_run_ = declare_parameter<bool>("dry_run", true);
    allow_live_motion_ = declare_parameter<bool>("allow_live_motion", false);
    startup_standup_ = declare_parameter<bool>("startup_standup", true);
    assume_standing_when_skip_standup_ =
        declare_parameter<bool>("assume_standing_when_skip_standup", false);
    local_ip_ = declare_parameter<std::string>("local_ip", env_or("AGIBOT_D1_LOCAL_IP", "192.168.168.100"));
    dog_ip_ = declare_parameter<std::string>("dog_ip", env_or("AGIBOT_D1_ROBOT_IP", "192.168.168.168"));
    local_port_ = declare_parameter<int>("local_port", env_int_or("AGIBOT_D1_LOCAL_PORT", 43988));
    policy_.vx.safe_max_abs = declare_parameter<double>("safe_max_vx_mps", policy_.vx.safe_max_abs);
    policy_.vy.safe_max_abs = declare_parameter<double>("safe_max_vy_mps", policy_.vy.safe_max_abs);
    policy_.yaw.safe_max_abs = declare_parameter<double>("safe_max_yaw_rate_radps", policy_.yaw.safe_max_abs);
    policy_.watchdog_timeout_s = declare_parameter<double>("watchdog_timeout_s", policy_.watchdog_timeout_s);
    policy_.stop_hold_s = declare_parameter<double>("stop_hold_s", policy_.stop_hold_s);
    policy_.startup_stand_wait_s = declare_parameter<double>("startup_stand_wait_s", policy_.startup_stand_wait_s);
    policy_.startup_stand_wait_s = std::max(policy_.startup_stand_wait_s, kMinimumStartupStandWaitS);
    policy_.startup_stand_confirm_timeout_s =
        declare_parameter<double>("startup_stand_confirm_timeout_s", policy_.startup_stand_confirm_timeout_s);
    policy_.startup_stand_confirm_poll_s =
        declare_parameter<double>("startup_stand_confirm_poll_s", policy_.startup_stand_confirm_poll_s);
    policy_.min_battery_percent_for_live_motion =
        declare_parameter<int>("min_battery_percent_for_live_motion", policy_.min_battery_percent_for_live_motion);

    status_publisher_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);
    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_, 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) { on_twist(*msg); });
    sdk_stream_timer_ = create_wall_timer(50ms, [this]() { on_sdk_stream_tick(); });

    live_enabled_ = compute_live_enabled();
    if (live_enabled_) {
      publish_status("starting");
      startup_timer_ = create_wall_timer(100ms, [this]() { start_live_control_session(); });
    } else {
      publish_status("ready");
    }
    RCLCPP_INFO(get_logger(), "Agibot D1 zsl-1 bridge topic=%s dry_run=%s",
                cmd_vel_topic_.c_str(), dry_run_ ? "true" : "false");
  }

  ~D1HighlevelBridgeNode() override {
    send_stop_frames();
  }

 private:
  static std::string env_or(const char *name, const char *fallback) {
    const char *value = std::getenv(name);
    return value == nullptr || std::string(value).empty() ? std::string(fallback) : std::string(value);
  }

  static int env_int_or(const char *name, int fallback) {
    const char *value = std::getenv(name);
    if (value == nullptr) {
      return fallback;
    }
    try {
      return std::stoi(value);
    } catch (...) {
      return fallback;
    }
  }

  bool compute_live_enabled() const {
    if (dry_run_) {
      return false;
    }
    if (!allow_live_motion_) {
      throw std::runtime_error("live motion requires allow_live_motion:=true");
    }
    const char *gate = std::getenv("AGIBOT_D1_ALLOW_LIVE_SDK");
    if (gate == nullptr || std::string(gate) != "1") {
      throw std::runtime_error("live motion requires AGIBOT_D1_ALLOW_LIVE_SDK=1");
    }
    return true;
  }

  void start_live_control_session() {
    if (startup_started_) {
      return;
    }
    startup_started_ = true;
    if (startup_timer_) {
      startup_timer_->cancel();
    }
    try {
      publish_status("starting_sdk");
      init_sdk();
      publish_status("starting_standup");
      startup_sequence();
      publish_status("ready");
    } catch (const std::exception &exc) {
      publish_status(std::string("startup_error:") + exc.what());
      RCLCPP_ERROR(get_logger(), "Agibot D1 live startup failed: %s", exc.what());
      throw;
    }
  }

  void init_sdk() {
    sdk_ = std::make_unique<mc_sdk::zsl_1::HighLevel>();
    sdk_->initRobot(local_ip_, local_port_, dog_ip_);
    std::this_thread::sleep_for(500ms);
    if (!sdk_->checkConnect()) {
      throw std::runtime_error("Agibot D1 SDK checkConnect failed");
    }
    const int battery = static_cast<int>(sdk_->getBatteryPower());
    const uint32_t ctrl_mode = sdk_->getCurrentCtrlmode();
    RCLCPP_INFO(get_logger(), "Agibot D1 SDK connected battery=%d ctrl_mode=%u", battery, ctrl_mode);
    if (battery < policy_.min_battery_percent_for_live_motion) {
      throw std::runtime_error("Agibot D1 battery below live motion threshold");
    }
  }

  void startup_sequence() {
    if (!sdk_) {
      return;
    }
    const uint32_t initial_ctrl_mode = sdk_->getCurrentCtrlmode();
    if (is_standing_ctrl_mode(initial_ctrl_mode)) {
      motion_ready_ = true;
      RCLCPP_INFO(get_logger(), "Agibot D1 already standing ctrl_mode=%u; skip standUp", initial_ctrl_mode);
      return;
    }
    if (initial_ctrl_mode == kMovingCtrlMode) {
      wait_without_move_command(std::chrono::milliseconds(
          static_cast<int>(policy_.startup_stand_confirm_poll_s * 1000.0)));
      const uint32_t settled_ctrl_mode = sdk_->getCurrentCtrlmode();
      if (is_standing_ctrl_mode(settled_ctrl_mode)) {
        motion_ready_ = true;
        RCLCPP_INFO(get_logger(), "Agibot D1 returned to standing ctrl_mode=%u; skip standUp", settled_ctrl_mode);
        return;
      }
      throw std::runtime_error(
          "Agibot D1 reports moving ctrl_mode=18 at startup; refusing automatic standUp from moving state");
    }
    if (!startup_standup_) {
      throw std::runtime_error(
          "Agibot D1 is not reporting standing and startup_standup:=false; set startup_standup:=true "
          "to call standUp automatically after the mandatory pre-stand wait");
    }
    wait_before_standup(initial_ctrl_mode);
    const uint32_t ret = sdk_->standUp();
    RCLCPP_INFO(get_logger(), "Agibot D1 standUp ret=%u from_ctrl_mode=%u post_wait_s=%.2f",
                ret, initial_ctrl_mode, policy_.startup_stand_wait_s);
    if (ret != 0) {
      throw std::runtime_error("Agibot D1 standUp failed before live motion");
    }
    wait_without_move_command(std::chrono::milliseconds(
        static_cast<int>(policy_.startup_stand_wait_s * 1000.0)));
    confirm_standing_after_standup();
  }

  void wait_before_standup(uint32_t ctrl_mode) {
    RCLCPP_INFO(get_logger(), "Agibot D1 pre-stand wait_s=%.2f from_ctrl_mode=%u",
                policy_.startup_stand_wait_s, ctrl_mode);
    wait_without_move_command(std::chrono::milliseconds(
        static_cast<int>(policy_.startup_stand_wait_s * 1000.0)));
  }

  void confirm_standing_after_standup() {
    const auto timeout = std::chrono::milliseconds(
        static_cast<int>(policy_.startup_stand_confirm_timeout_s * 1000.0));
    const auto poll = std::chrono::milliseconds(
        std::max(1, static_cast<int>(policy_.startup_stand_confirm_poll_s * 1000.0)));
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    uint32_t last_ctrl_mode = sdk_->getCurrentCtrlmode();

    while (std::chrono::steady_clock::now() <= deadline) {
      last_ctrl_mode = sdk_->getCurrentCtrlmode();
      if (is_standing_ctrl_mode(last_ctrl_mode)) {
        motion_ready_ = true;
        RCLCPP_INFO(get_logger(), "Agibot D1 standing confirmed ctrl_mode=%u", last_ctrl_mode);
        return;
      }
      wait_without_move_command(poll);
    }

    throw std::runtime_error(
        "Agibot D1 standUp did not reach standing ctrl_mode=1; last_ctrl_mode=" +
        std::to_string(last_ctrl_mode));
  }

  std::chrono::milliseconds sdk_frame_period() const {
    return std::chrono::milliseconds(
        std::max(1, static_cast<int>(1000.0 / policy_.publish_rate_hz)));
  }

  void wait_without_move_command(std::chrono::milliseconds duration) {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    const auto period = std::chrono::milliseconds(
        std::max(1, static_cast<int>(policy_.startup_stand_confirm_poll_s * 1000.0)));
    while (std::chrono::steady_clock::now() < deadline) {
      const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
          deadline - std::chrono::steady_clock::now());
      std::this_thread::sleep_for(std::min(period, std::max(1ms, remaining)));
    }
  }

  void on_twist(const geometry_msgs::msg::Twist &msg) {
    if (sdk_move_fault_) {
      current_command_ = stop_command();
      publish_status("sdk_move_fault_blocked");
      return;
    }
    const CommandDecision decision = normalize_twist(policy_, msg.linear.x, msg.linear.y, msg.angular.z);
    if (!decision.allowed) {
      RCLCPP_WARN(get_logger(), "%s", decision.reason.c_str());
      publish_status(decision.reason);
      current_command_ = stop_command();
      send_sdk_move(current_command_);
      return;
    }

    last_command_time_ = now();
    current_command_ = decision.command;
    publish_status("accepted");
    const bool nonzero =
        current_command_.vx_mps != 0.0 || current_command_.vy_mps != 0.0 || current_command_.yaw_rate_radps != 0.0;
    if (nonzero) {
      RCLCPP_INFO(get_logger(), "accepted vx=%.3f vy=%.3f yaw=%.3f",
                  current_command_.vx_mps, current_command_.vy_mps, current_command_.yaw_rate_radps);
    }
  }

  void on_sdk_stream_tick() {
    const double age_s = (now() - last_command_time_).seconds();
    if (age_s > policy_.watchdog_timeout_s) {
      current_command_ = stop_command();
      publish_status("watchdog_stop");
    }
    send_sdk_move(current_command_);
  }

  void send_sdk_move(const MoveCommand &command) {
    if (!live_enabled_ || !sdk_) {
      return;
    }
    if (!motion_ready_) {
      return;
    }
    const bool nonzero = command.vx_mps != 0.0 || command.vy_mps != 0.0 || command.yaw_rate_radps != 0.0;
    if (sdk_move_fault_ && nonzero) {
      return;
    }
    const uint32_t ret = sdk_->move(
        static_cast<float>(command.vx_mps),
        static_cast<float>(command.vy_mps),
        static_cast<float>(command.yaw_rate_radps));
    if (ret != 0) {
      if (nonzero) {
        sdk_move_fault_ = true;
        current_command_ = stop_command();
      }
      if (nonzero || !zero_move_error_logged_) {
        RCLCPP_WARN(get_logger(), "Agibot D1 move ret=%u vx=%.3f vy=%.3f yaw=%.3f",
                    ret, command.vx_mps, command.vy_mps, command.yaw_rate_radps);
      }
      if (!nonzero) {
        zero_move_error_logged_ = true;
      }
      publish_status("sdk_move_error:" + std::to_string(ret));
      return;
    }
    if (nonzero && !first_live_move_logged_) {
      first_live_move_logged_ = true;
      RCLCPP_INFO(get_logger(), "Agibot D1 live move accepted by SDK vx=%.3f vy=%.3f yaw=%.3f",
                  command.vx_mps, command.vy_mps, command.yaw_rate_radps);
    }
  }

  void send_stop_frames() {
    if (!live_enabled_ || !sdk_) {
      return;
    }
    const MoveCommand stop = stop_command();
    const int frames = stop_frame_count(policy_);
    for (int i = 0; i < frames; ++i) {
      send_sdk_move(stop);
      std::this_thread::sleep_for(std::chrono::milliseconds(
          static_cast<int>(1000.0 / policy_.publish_rate_hz)));
    }
  }

  void publish_status(const std::string &status) {
    if (!status_publisher_) {
      return;
    }
    std_msgs::msg::String msg;
    msg.data = status;
    status_publisher_->publish(msg);
  }

  BridgePolicy policy_{};
  bool dry_run_{true};
  bool allow_live_motion_{false};
  bool startup_standup_{true};
  bool assume_standing_when_skip_standup_{false};
  bool live_enabled_{false};
  bool startup_started_{false};
  bool motion_ready_{false};
  bool first_live_move_logged_{false};
  bool sdk_move_fault_{false};
  bool zero_move_error_logged_{false};
  int local_port_{43988};
  std::string local_ip_;
  std::string dog_ip_;
  std::string cmd_vel_topic_;
  std::string status_topic_;
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  MoveCommand current_command_{};
  std::unique_ptr<mc_sdk::zsl_1::HighLevel> sdk_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::TimerBase::SharedPtr sdk_stream_timer_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
};

}  // namespace agibot_d1_ros

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<agibot_d1_ros::D1HighlevelBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
