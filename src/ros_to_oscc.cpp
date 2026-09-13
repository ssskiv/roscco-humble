#include "roscco/ros_to_oscc.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

#include "roscco/parameter_utils.hpp"

namespace roscco
{

RosToOscc::RosToOscc(rclcpp::Node * node)
: node_(node)
{
  // --- command limits --------------------------------------------------------
  brake_min_ = declareDouble(
    node_, "limits.brake_min", 0.0, 0.0, 1.0,
    "Lower clamp for brake_command.brake_position.");
  brake_max_ = declareDouble(
    node_, "limits.brake_max", 1.0, 0.0, 1.0,
    "Upper clamp for brake_command.brake_position. Lower this while bringing "
    "the vehicle up.");
  throttle_min_ = declareDouble(
    node_, "limits.throttle_min", 0.0, 0.0, 1.0,
    "Lower clamp for throttle_command.throttle_position.");
  throttle_max_ = declareDouble(
    node_, "limits.throttle_max", 1.0, 0.0, 1.0,
    "Upper clamp for throttle_command.throttle_position.");
  steering_min_ = declareDouble(
    node_, "limits.steering_torque_min", -1.0, -1.0, 0.0,
    "Lower clamp for steering_command.steering_torque.");
  steering_max_ = declareDouble(
    node_, "limits.steering_torque_max", 1.0, 0.0, 1.0,
    "Upper clamp for steering_command.steering_torque.");

  if (brake_min_ > brake_max_ || throttle_min_ > throttle_max_ ||
    steering_min_ > steering_max_)
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "A limits.* minimum exceeds its maximum; every command will be pinned to "
      "the maximum. Fix the parameters.");
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "Command limits: brake [%.2f, %.2f] throttle [%.2f, %.2f] steering [%.2f, %.2f]",
    brake_min_, brake_max_, throttle_min_, throttle_max_, steering_min_, steering_max_);

  // --- subscriptions ---------------------------------------------------------
  sigset_t mask;
  sigset_t orig_mask;

  sigemptyset(&mask);
  sigemptyset(&orig_mask);
  sigaddset(&mask, SIGIO);

  // Block OSCC's SIGIO while the subscriptions are being built.
  if (sigprocmask(SIG_BLOCK, &mask, &orig_mask) < 0) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to block SIGIO");
  }

  const rclcpp::QoS qos(10);

  brake_sub_ = node_->create_subscription<msg::BrakeCommand>(
    "brake_command", qos,
    std::bind(&RosToOscc::brakeCommandCallback, this, std::placeholders::_1));

  steering_sub_ = node_->create_subscription<msg::SteeringCommand>(
    "steering_command", qos,
    std::bind(&RosToOscc::steeringCommandCallback, this, std::placeholders::_1));

  throttle_sub_ = node_->create_subscription<msg::ThrottleCommand>(
    "throttle_command", qos,
    std::bind(&RosToOscc::throttleCommandCallback, this, std::placeholders::_1));

  enable_disable_sub_ = node_->create_subscription<msg::EnableDisable>(
    "enable_disable", qos,
    std::bind(&RosToOscc::enableDisableCallback, this, std::placeholders::_1));

  if (sigprocmask(SIG_SETMASK, &orig_mask, nullptr) < 0) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to unblock SIGIO");
  }
}

double RosToOscc::clamp(double value, double lo, double hi, const char * what)
{
  // A NaN reaching the firmware as a float is worse than a zero command.
  if (!std::isfinite(value)) {
    RCLCPP_ERROR(node_->get_logger(), "Non-finite %s command; substituting 0.", what);
    return std::clamp(0.0, lo, hi);
  }

  const double clamped = std::clamp(value, lo, hi);

  if (clamped != value) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "%s command %.3f clamped to %.3f by limits [%.2f, %.2f]",
      what, value, clamped, lo, hi);
  }

  return clamped;
}

bool RosToOscc::report(oscc_result_t result, const char * action)
{
  if (result == OSCC_ERROR) {
    RCLCPP_ERROR(node_->get_logger(), "OSCC_ERROR while trying to %s", action);
    return false;
  }

  if (result == OSCC_WARNING) {
    RCLCPP_WARN(node_->get_logger(), "OSCC_WARNING while trying to %s", action);
    return false;
  }

  return true;
}

void RosToOscc::brakeCommandCallback(const msg::BrakeCommand::SharedPtr message)
{
  const double value = clamp(message->brake_position, brake_min_, brake_max_, "brake");
  report(oscc_publish_brake_position(value), "send the brake position");
}

void RosToOscc::steeringCommandCallback(const msg::SteeringCommand::SharedPtr message)
{
  const double value = clamp(message->steering_torque, steering_min_, steering_max_, "steering");
  report(oscc_publish_steering_torque(value), "send the steering torque");
}

void RosToOscc::throttleCommandCallback(const msg::ThrottleCommand::SharedPtr message)
{
  const double value = clamp(message->throttle_position, throttle_min_, throttle_max_, "throttle");
  report(oscc_publish_throttle_position(value), "send the throttle position");
}

void RosToOscc::enableDisableCallback(const msg::EnableDisable::SharedPtr message)
{
  const bool enable = message->enable_control;

  if (report(enable ? oscc_enable() : oscc_disable(), "enable or disable control")) {
    RCLCPP_INFO(node_->get_logger(), "OSCC control %s", enable ? "ENABLED" : "disabled");
  }
}

}  // namespace roscco
