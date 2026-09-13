#include "roscco/ros_to_oscc.hpp"

#include <functional>

namespace roscco
{

RosToOscc::RosToOscc(rclcpp::Node * node)
: node_(node)
{
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
  report(oscc_publish_brake_position(message->brake_position), "send the brake position");
}

void RosToOscc::steeringCommandCallback(const msg::SteeringCommand::SharedPtr message)
{
  report(oscc_publish_steering_torque(message->steering_torque), "send the steering torque");
}

void RosToOscc::throttleCommandCallback(const msg::ThrottleCommand::SharedPtr message)
{
  report(oscc_publish_throttle_position(message->throttle_position), "send the throttle position");
}

void RosToOscc::enableDisableCallback(const msg::EnableDisable::SharedPtr message)
{
  const bool enable = message->enable_control;

  if (report(enable ? oscc_enable() : oscc_disable(), "enable or disable control")) {
    RCLCPP_INFO(node_->get_logger(), "OSCC control %s", enable ? "ENABLED" : "disabled");
  }
}

}  // namespace roscco
