// Republish Gazebo's simulation clock at a bounded rate.
//
// Gazebo publishes /clock once per 1 ms physics step (~830 Hz in practice).
// Every node using simulation time deserializes every message, which costs a
// Python node about a third of a CPU core. Navigation needs far less resolution
// (controllers run at 20 Hz), so this node forwards one clock message per
// `period_ms` of *simulation* time, independent of the real-time factor.
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

class ClockThrottle : public rclcpp::Node
{
public:
  ClockThrottle()
  : Node("clock_throttle")
  {
    const auto period_ms = declare_parameter<int64_t>("period_ms", 10);
    period_ns_ = period_ms * 1000000LL;
    publisher_ = create_publisher<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    subscription_ = create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock_raw", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
      [this](rosgraph_msgs::msg::Clock::ConstSharedPtr msg) {
        const int64_t now = static_cast<int64_t>(msg->clock.sec) * 1000000000LL + msg->clock.nanosec;
        // Forward the first message, every period, and any backwards jump (simulation reset).
        if (last_ns_ < 0 || now - last_ns_ >= period_ns_ || now < last_ns_) {
          last_ns_ = now;
          publisher_->publish(*msg);
        }
      });
    RCLCPP_INFO(get_logger(), "Forwarding /clock_raw to /clock every %ld ms of simulation time", period_ms);
  }

private:
  int64_t period_ns_{10000000};
  int64_t last_ns_{-1};
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr publisher_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClockThrottle>());
  rclcpp::shutdown();
  return 0;
}
