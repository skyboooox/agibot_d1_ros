#include <cstdlib>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "zsl-1/highlevel.h"

using namespace std::chrono_literals;

namespace agibot_d1_ros {

class D1StateNode final : public rclcpp::Node {
 public:
  D1StateNode() : Node("agibot_d1_state_node") {
    state_topic_ = declare_parameter<std::string>("state_topic", "/agibot_d1/state");
    dry_run_ = declare_parameter<bool>("dry_run", true);
    allow_sdk_connect_ = declare_parameter<bool>("allow_sdk_connect", false);
    allow_watchdog_damping_risk_ = declare_parameter<bool>("allow_watchdog_damping_risk", false);
    local_ip_ = declare_parameter<std::string>("local_ip", env_or("AGIBOT_D1_LOCAL_IP", "192.168.168.100"));
    dog_ip_ = declare_parameter<std::string>("dog_ip", env_or("AGIBOT_D1_ROBOT_IP", "192.168.168.168"));
    local_port_ = declare_parameter<int>("local_port", env_int_or("AGIBOT_D1_LOCAL_PORT", 43988));
    publisher_ = create_publisher<std_msgs::msg::String>(state_topic_, 10);

    if (live_readonly_enabled()) {
      sdk_ = std::make_unique<mc_sdk::zsl_1::HighLevel>();
      sdk_->initRobot(local_ip_, local_port_, dog_ip_);
    }

    timer_ = create_wall_timer(200ms, [this]() { publish_state(); });
    RCLCPP_INFO(get_logger(), "Agibot D1 state topic=%s dry_run=%s", state_topic_.c_str(), dry_run_ ? "true" : "false");
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

  bool live_readonly_enabled() const {
    if (dry_run_) {
      return false;
    }
    if (!allow_sdk_connect_) {
      throw std::runtime_error("read-only SDK connect requires allow_sdk_connect:=true");
    }
    const char *gate = std::getenv("AGIBOT_D1_ALLOW_READONLY_SDK");
    if (gate == nullptr || std::string(gate) != "1") {
      throw std::runtime_error("read-only SDK connect requires AGIBOT_D1_ALLOW_READONLY_SDK=1");
    }
    const char *risk_gate = std::getenv("AGIBOT_D1_ALLOW_WATCHDOG_DAMPING_RISK");
    if (!allow_watchdog_damping_risk_ || risk_gate == nullptr || std::string(risk_gate) != "1") {
      throw std::runtime_error(
          "read-only SDK connect can seize SDK control and trigger watchdog damping; "
          "use robot ROS/eCAL state topics instead, or explicitly set "
          "allow_watchdog_damping_risk:=true and AGIBOT_D1_ALLOW_WATCHDOG_DAMPING_RISK=1");
    }
    return true;
  }

  void publish_state() {
    std_msgs::msg::String msg;
    std::ostringstream out;
    out << "{\"dry_run\":" << (dry_run_ ? "true" : "false")
        << ",\"direct_sdk_exposed\":false"
        << ",\"robot\":\"agibot_d1_ultra_edu\""
        << ",\"variant\":\"zsl-1\"";
    if (!dry_run_ && sdk_) {
      const bool connected = sdk_->checkConnect();
      out << ",\"connected\":" << (connected ? "true" : "false");
      if (connected) {
        const auto position = sdk_->getPosition();
        const auto world_velocity = sdk_->getWorldVelocity();
        const auto rpy = sdk_->getRPY();
        out << ",\"battery_power\":" << sdk_->getBatteryPower()
            << ",\"ctrl_mode\":" << sdk_->getCurrentCtrlmode()
            << ",\"position\":" << json_array(position)
            << ",\"world_velocity\":" << json_array(world_velocity)
            << ",\"rpy\":" << json_array(rpy);
      }
    } else {
      out << ",\"connected\":false,\"battery_power\":null,\"ctrl_mode\":null";
    }
    out << "}";
    msg.data = out.str();
    publisher_->publish(msg);
  }

  static std::string json_array(const std::vector<float> &values) {
    std::ostringstream out;
    out << "[";
    for (std::size_t i = 0; i < values.size(); ++i) {
      if (i != 0) {
        out << ",";
      }
      out << values[i];
    }
    out << "]";
    return out.str();
  }

  bool dry_run_{true};
  bool allow_sdk_connect_{false};
  bool allow_watchdog_damping_risk_{false};
  int local_port_{43988};
  std::string local_ip_;
  std::string dog_ip_;
  std::string state_topic_;
  std::unique_ptr<mc_sdk::zsl_1::HighLevel> sdk_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace agibot_d1_ros

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<agibot_d1_ros::D1StateNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
