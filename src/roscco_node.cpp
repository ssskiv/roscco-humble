#include <memory>

extern "C" {
#include <oscc.h>
}

#include <rclcpp/rclcpp.hpp>

#include "roscco/oscc_to_ros.hpp"
#include "roscco/ros_to_oscc.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("roscco_node");

  const int can_channel = node->declare_parameter<int>("can_channel", 0);
  const int drain_period_ms = node->declare_parameter<int>("drain_period_ms", 5);

  // NOTE: older OSCC API revisions expose oscc_init() with no arguments
  // instead of oscc_open(channel). If your checkout of the oscc submodule
  // predates the channel argument, swap the line below for oscc_init().
  if (oscc_open(static_cast<unsigned int>(can_channel)) != OSCC_OK) {
    RCLCPP_FATAL(
      node->get_logger(),
      "Could not open OSCC on can%d. Is the interface up? "
      "(sudo ip link set can%d up type can bitrate 500000)",
      can_channel, can_channel);
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "OSCC opened on can%d", can_channel);

  // Order matters: start publishing reports before accepting commands.
  auto publisher = std::make_unique<roscco::OsccToRos>(node.get(), drain_period_ms);
  auto subscriber = std::make_unique<roscco::RosToOscc>(node.get());

  rclcpp::spin(node);

  // Reached on SIGINT. Disable first so the modules stop actuating, then close.
  if (oscc_disable() != OSCC_OK) {
    RCLCPP_ERROR(node->get_logger(), "Could not disable OSCC");
  }

  publisher.reset();
  subscriber.reset();

  if (oscc_close(static_cast<unsigned int>(can_channel)) != OSCC_OK) {
    RCLCPP_ERROR(node->get_logger(), "Could not close OSCC connection");
  }

  rclcpp::shutdown();
  return 0;
}
