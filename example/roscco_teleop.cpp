#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "roscco/parameter_utils.hpp"

#include "roscco/msg/brake_command.hpp"
#include "roscco/msg/enable_disable.hpp"
#include "roscco/msg/steering_command.hpp"
#include "roscco/msg/throttle_command.hpp"

namespace
{

double exponentialAverage(double average, double setpoint, double factor)
{
  return (setpoint * factor) + ((1.0 - factor) * average);
}

double linearTransform(double value, double high_1, double low_1, double high_2, double low_2)
{
  return low_2 + (value - low_1) * (high_2 - low_2) / (high_1 - low_1);
}

}  // namespace

/**
 * @brief Gamepad teleop for ROSCCO.
 *
 * Differs from the ROS 1 example in two ways that matter on a real vehicle:
 *
 *  1. Commands are published from a fixed 50 Hz timer, not from the joy
 *     callback. The OSCC modules fault out if they go 200 ms without a
 *     command, and joy_node's rate is not guaranteed.
 *  2. A watchdog zeroes the commands and sends a disable if joy messages stop
 *     arriving. Unplugging the gamepad used to leave the last command latched
 *     until the firmware timeout fired.
 *
 * Axis and button indices are parameters rather than constants -- the defaults
 * match a Logitech F310 / wired Xbox pad in XInput mode.
 */
class RosccoTeleop : public rclcpp::Node
{
public:
  RosccoTeleop()
  : Node("roscco_teleop")
  {
    using roscco::declareDouble;
    using roscco::declareInt;

    brake_axis_ = static_cast<int>(declareInt(
      this, "brake_axis", 2, 0, 31, "Joy axis index for the brake trigger."));
    throttle_axis_ = static_cast<int>(declareInt(
      this, "throttle_axis", 5, 0, 31, "Joy axis index for the throttle trigger."));
    steering_axis_ = static_cast<int>(declareInt(
      this, "steering_axis", 0, 0, 31, "Joy axis index for the steering stick."));
    start_button_ = static_cast<int>(declareInt(
      this, "start_button", 7, 0, 31, "Joy button index that enables control."));
    back_button_ = static_cast<int>(declareInt(
      this, "back_button", 6, 0, 31, "Joy button index that disables control."));

    smoothing_factor_ = declareDouble(
      this, "steering_smoothing_factor", 0.1, 0.01, 1.0,
      "Exponential-average coefficient. 1.0 disables smoothing.");
    publish_rate_hz_ = declareDouble(
      this, "publish_rate_hz", 50.0, 5.0, 200.0,
      "Command publish rate. Must stay above 5 Hz: the OSCC modules fault out "
      "after 200 ms without a command.");
    joy_timeout_s_ = declareDouble(
      this, "joy_timeout", 0.3, 0.05, 5.0,
      "Seconds without a joy message before commands are zeroed and control "
      "is disabled.");

    max_brake_ = declareDouble(
      this, "max_brake", 1.0, 0.0, 1.0, "Scales the brake trigger output.");
    max_throttle_ = declareDouble(
      this, "max_throttle", 1.0, 0.0, 1.0, "Scales the throttle trigger output.");
    max_steering_ = declareDouble(
      this, "max_steering_torque", 1.0, 0.0, 1.0, "Scales the steering stick output.");

    const rclcpp::QoS qos(10);

    brake_pub_ = create_publisher<roscco::msg::BrakeCommand>("brake_command", qos);
    throttle_pub_ = create_publisher<roscco::msg::ThrottleCommand>("throttle_command", qos);
    steering_pub_ = create_publisher<roscco::msg::SteeringCommand>("steering_command", qos);
    enable_disable_pub_ = create_publisher<roscco::msg::EnableDisable>("enable_disable", qos);

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", qos, std::bind(&RosccoTeleop::joyCallback, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&RosccoTeleop::publishCommands, this));

    RCLCPP_INFO(get_logger(), "Pull both triggers fully to arm, then press START to enable.");
  }

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
  {
    const int max_axis = std::max({brake_axis_, throttle_axis_, steering_axis_});
    const int max_button = std::max(start_button_, back_button_);

    if (static_cast<int>(joy->axes.size()) <= max_axis ||
      static_cast<int>(joy->buttons.size()) <= max_button)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Joy message has %zu axes / %zu buttons -- not enough for the configured indices. "
        "Is the F310 switch on 'X' rather than 'D'?",
        joy->axes.size(), joy->buttons.size());
      return;
    }

    last_joy_ = now();

    // Triggers report 0.0 until they are first moved, which reads as 50%.
    // Require both to be pulled to the parked end before accepting input.
    if (!armed_) {
      if (joy->axes[brake_axis_] > kParkedThreshold &&
        joy->axes[throttle_axis_] > kParkedThreshold)
      {
        armed_ = true;
        RCLCPP_INFO(get_logger(), "Triggers armed.");
      } else {
        if (joy->axes[brake_axis_] <= kParkedThreshold) {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Pull the brake trigger.");
        }
        if (joy->axes[throttle_axis_] <= kParkedThreshold) {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Pull the throttle trigger.");
        }
      }
      return;
    }

    // Triggers: [1, -1] -> [0, 1]. Steering stick: [1, -1] -> [-1, 1].
    brake_ = max_brake_ *
      linearTransform(joy->axes[brake_axis_], kTriggerMax, kTriggerMin, 1.0, 0.0);
    throttle_ = max_throttle_ *
      linearTransform(joy->axes[throttle_axis_], kTriggerMax, kTriggerMin, 1.0, 0.0);
    steering_ = max_steering_ *
      linearTransform(joy->axes[steering_axis_], kStickMax, kStickMin, 1.0, -1.0);

    if (previous_back_ == 0 && joy->buttons[back_button_]) {
      setEnabled(false);
    } else if (previous_start_ == 0 && joy->buttons[start_button_]) {
      setEnabled(true);
    }

    previous_back_ = joy->buttons[back_button_];
    previous_start_ = joy->buttons[start_button_];
  }

  void setEnabled(bool enable)
  {
    roscco::msg::EnableDisable message;
    message.header.stamp = now();
    message.enable_control = enable;
    enable_disable_pub_->publish(message);
    enabled_ = enable;

    RCLCPP_INFO(get_logger(), "Control %s", enable ? "ENABLED" : "disabled");
  }

  void publishCommands()
  {
    // Watchdog: joy_node died, gamepad unplugged, or the topic went quiet.
    if (enabled_ && last_joy_.nanoseconds() > 0 &&
      (now() - last_joy_).seconds() > joy_timeout_s_)
    {
      RCLCPP_ERROR(get_logger(), "Joy input timed out -- zeroing commands and disabling.");
      brake_ = 0.0;
      throttle_ = 0.0;
      steering_ = 0.0;
      steering_average_ = 0.0;
      armed_ = false;
      setEnabled(false);
    }

    if (!enabled_) {
      return;
    }

    const auto stamp = now();

    roscco::msg::BrakeCommand brake_message;
    brake_message.header.stamp = stamp;
    brake_message.brake_position = brake_;
    brake_pub_->publish(brake_message);

    roscco::msg::ThrottleCommand throttle_message;
    throttle_message.header.stamp = stamp;
    throttle_message.throttle_position = throttle_;
    throttle_pub_->publish(throttle_message);

    steering_average_ = exponentialAverage(steering_average_, steering_, smoothing_factor_);

    roscco::msg::SteeringCommand steering_message;
    steering_message.header.stamp = stamp;
    steering_message.steering_torque = steering_average_;
    steering_pub_->publish(steering_message);
  }

  static constexpr double kParkedThreshold = 0.99;
  static constexpr double kTriggerMin = 1.0;
  static constexpr double kTriggerMax = -1.0;
  static constexpr double kStickMin = 1.0;
  static constexpr double kStickMax = -1.0;

  rclcpp::Publisher<roscco::msg::BrakeCommand>::SharedPtr brake_pub_;
  rclcpp::Publisher<roscco::msg::ThrottleCommand>::SharedPtr throttle_pub_;
  rclcpp::Publisher<roscco::msg::SteeringCommand>::SharedPtr steering_pub_;
  rclcpp::Publisher<roscco::msg::EnableDisable>::SharedPtr enable_disable_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  int brake_axis_{2};
  int throttle_axis_{5};
  int steering_axis_{0};
  int start_button_{7};
  int back_button_{6};

  double smoothing_factor_{0.1};
  double publish_rate_hz_{50.0};
  double joy_timeout_s_{0.3};
  double max_brake_{1.0};
  double max_throttle_{1.0};
  double max_steering_{1.0};

  double brake_{0.0};
  double throttle_{0.0};
  double steering_{0.0};
  double steering_average_{0.0};

  bool armed_{false};
  bool enabled_{false};
  int previous_start_{0};
  int previous_back_{0};
  rclcpp::Time last_joy_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RosccoTeleop>());
  rclcpp::shutdown();
  return 0;
}
