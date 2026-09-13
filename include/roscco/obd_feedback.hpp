#ifndef ROSCCO__OBD_FEEDBACK_HPP_
#define ROSCCO__OBD_FEEDBACK_HPP_

#include <cstdint>
#include <string>

#include <linux/can.h>

#include <rclcpp/rclcpp.hpp>

#include "roscco/msg/brake_pedal_report.hpp"
#include "roscco/msg/steering_angle_report.hpp"

namespace roscco
{

/**
 * @brief A DBC-style signal inside a CAN frame.
 *
 * Deliberately generic: OSCC only supports the Kia Soul upstream, and the OBD
 * IDs of any other vehicle have to be reverse-engineered. Rather than hardcode
 * a vehicle, every field here is a ROS parameter.
 */
struct SignalSpec
{
  bool enabled{false};
  std::uint32_t can_id{0};
  int start_bit{0};       ///< Intel: LSB position. Motorola: MSB position.
  int bit_length{16};
  bool little_endian{true};
  bool is_signed{true};
  double scale{1.0};
  double offset{0.0};

  /// Decode the raw (unscaled) integer out of a frame payload.
  std::int64_t decodeRaw(const std::uint8_t * data, std::uint8_t dlc) const;

  /// Decode and apply scale/offset.
  double decode(const std::uint8_t * data, std::uint8_t dlc) const
  {
    return static_cast<double>(decodeRaw(data, dlc)) * scale + offset;
  }

  /// True if this spec can actually be read out of a frame of the given length.
  bool fitsIn(std::uint8_t dlc) const;

  std::string summary() const;
};

/**
 * @brief Turns raw vehicle CAN frames into steering-angle and brake-pedal topics.
 *
 * Fed from the OBD frames that the CAN gateway republishes onto the control
 * bus. Without a gateway module on the bus no frames arrive and both topics
 * stay silent -- that is expected, not a fault.
 */
class ObdFeedback
{
public:
  explicit ObdFeedback(rclcpp::Node * node);

  /// Offer a frame; publishes if it matches a configured ID.
  void process(const struct can_frame & frame, const rclcpp::Time & stamp);

  /// Called periodically to warn when a configured signal has gone quiet.
  void checkStaleness(const rclcpp::Time & now);

private:
  rclcpp::Node * node_;

  SignalSpec steering_;
  double steering_min_{-720.0};
  double steering_max_{720.0};

  SignalSpec brake_pedal_;
  double brake_press_threshold_{0.5};
  bool brake_active_high_{true};

  double feedback_timeout_{1.0};

  rclcpp::Publisher<msg::SteeringAngleReport>::SharedPtr steering_pub_;
  rclcpp::Publisher<msg::BrakePedalReport>::SharedPtr brake_pub_;

  rclcpp::Time last_steering_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_brake_{0, 0, RCL_ROS_TIME};
};

}  // namespace roscco

#endif  // ROSCCO__OBD_FEEDBACK_HPP_
