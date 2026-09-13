#ifndef ROSCCO__ROS_TO_OSCC_HPP_
#define ROSCCO__ROS_TO_OSCC_HPP_

#include <csignal>

extern "C" {
#include <oscc.h>
}

#include <rclcpp/rclcpp.hpp>

#include "roscco/msg/brake_command.hpp"
#include "roscco/msg/enable_disable.hpp"
#include "roscco/msg/steering_command.hpp"
#include "roscco/msg/throttle_command.hpp"

namespace roscco
{

/// Forwards ROS 2 command topics into the OSCC API.
class RosToOscc
{
public:
  explicit RosToOscc(rclcpp::Node * node);

  RosToOscc(const RosToOscc &) = delete;
  RosToOscc & operator=(const RosToOscc &) = delete;

private:
  void brakeCommandCallback(const msg::BrakeCommand::SharedPtr message);
  void steeringCommandCallback(const msg::SteeringCommand::SharedPtr message);
  void throttleCommandCallback(const msg::ThrottleCommand::SharedPtr message);
  void enableDisableCallback(const msg::EnableDisable::SharedPtr message);

  /// Logs OSCC_ERROR / OSCC_WARNING uniformly. Returns true on OSCC_OK.
  bool report(oscc_result_t result, const char * action);

  rclcpp::Node * node_;

  rclcpp::Subscription<msg::BrakeCommand>::SharedPtr brake_sub_;
  rclcpp::Subscription<msg::SteeringCommand>::SharedPtr steering_sub_;
  rclcpp::Subscription<msg::ThrottleCommand>::SharedPtr throttle_sub_;
  rclcpp::Subscription<msg::EnableDisable>::SharedPtr enable_disable_sub_;
};

}  // namespace roscco

#endif  // ROSCCO__ROS_TO_OSCC_HPP_
