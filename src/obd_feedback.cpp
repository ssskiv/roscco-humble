#include "roscco/obd_feedback.hpp"

#include <algorithm>
#include <sstream>

#include "roscco/parameter_utils.hpp"

namespace roscco
{

namespace
{

/// Mask off the RTR/EFF/ERR flag bits so comparisons use the plain identifier.
std::uint32_t plainId(const struct can_frame & frame)
{
  return (frame.can_id & CAN_EFF_FLAG) ? (frame.can_id & CAN_EFF_MASK)
                                       : (frame.can_id & CAN_SFF_MASK);
}

}  // namespace

bool SignalSpec::fitsIn(std::uint8_t dlc) const
{
  if (bit_length < 1 || bit_length > 64 || start_bit < 0 || start_bit > 63) {
    return false;
  }

  const int available = static_cast<int>(std::min<std::uint8_t>(dlc, 8)) * 8;

  if (little_endian) {
    return start_bit + bit_length <= available;
  }

  // Motorola: the signal walks down from start_bit, so it only needs the byte
  // the start bit is in and the bytes that follow.
  const int start_byte = start_bit / 8;
  const int needed_bytes = start_byte + ((bit_length + 7) / 8);
  return needed_bytes * 8 <= available + 7 && start_byte * 8 < available;
}

std::int64_t SignalSpec::decodeRaw(const std::uint8_t * data, std::uint8_t dlc) const
{
  const int usable = static_cast<int>(std::min<std::uint8_t>(dlc, 8));
  std::uint64_t value = 0;

  if (little_endian) {
    std::uint64_t packed = 0;
    for (int i = 0; i < usable; ++i) {
      packed |= static_cast<std::uint64_t>(data[i]) << (8 * i);
    }
    const std::uint64_t mask =
      (bit_length == 64) ? ~0ULL : ((1ULL << bit_length) - 1ULL);
    value = (packed >> start_bit) & mask;
  } else {
    // Motorola / big-endian sawtooth bit order, same convention as DBC files.
    int bit = start_bit;
    for (int i = 0; i < bit_length; ++i) {
      const int byte_index = bit / 8;
      if (byte_index >= usable || byte_index < 0) {
        break;
      }
      const int bit_in_byte = bit % 8;
      value = (value << 1) | ((data[byte_index] >> bit_in_byte) & 1U);
      bit = (bit % 8 == 0) ? bit + 15 : bit - 1;
    }
  }

  if (is_signed && bit_length < 64) {
    const std::uint64_t sign_bit = 1ULL << (bit_length - 1);
    if (value & sign_bit) {
      value |= ~((1ULL << bit_length) - 1ULL);  // sign-extend
    }
  }

  return static_cast<std::int64_t>(value);
}

std::string SignalSpec::summary() const
{
  std::ostringstream out;
  out << "id=0x" << std::hex << can_id << std::dec
      << " bits=" << start_bit << "+" << bit_length
      << (little_endian ? " intel" : " motorola")
      << (is_signed ? " signed" : " unsigned")
      << " scale=" << scale << " offset=" << offset;
  return out.str();
}

ObdFeedback::ObdFeedback(rclcpp::Node * node)
: node_(node)
{
  // --- steering wheel angle --------------------------------------------------
  steering_.enabled = declareBool(
    node_, "steering_feedback.enabled", true,
    "Decode steering wheel angle from OBD frames forwarded by the CAN gateway.");
  steering_.can_id = static_cast<std::uint32_t>(declareInt(
    node_, "steering_feedback.can_id", 0x2B0, 0, 0x1FFFFFFF,
    "CAN identifier carrying the steering wheel angle. Default is the Kia Soul "
    "value from OSCC's vehicles.h -- set it for your own vehicle."));
  steering_.start_bit = static_cast<int>(declareInt(
    node_, "steering_feedback.start_bit", 0, 0, 63,
    "Position of the signal. Intel: LSB position. Motorola: MSB position."));
  steering_.bit_length = static_cast<int>(declareInt(
    node_, "steering_feedback.bit_length", 16, 1, 64,
    "Signal width in bits."));
  steering_.little_endian = declareBool(
    node_, "steering_feedback.little_endian", true,
    "True for Intel byte order, false for Motorola.");
  steering_.is_signed = declareBool(
    node_, "steering_feedback.is_signed", true,
    "Interpret the raw value as two's complement.");
  steering_.scale = declareDouble(
    node_, "steering_feedback.scale", 0.1, -1000.0, 1000.0,
    "Multiplier applied to the raw value. 0.1 gives degrees on the Kia Soul.");
  steering_.offset = declareDouble(
    node_, "steering_feedback.offset", 0.0, -100000.0, 100000.0,
    "Added after scaling.");

  steering_min_ = declareDouble(
    node_, "steering_feedback.min_angle", -720.0, -10000.0, 0.0,
    "Lower plausibility bound. Outside it the report is published with "
    "in_range=false instead of being suppressed.");
  steering_max_ = declareDouble(
    node_, "steering_feedback.max_angle", 720.0, 0.0, 10000.0,
    "Upper plausibility bound.");

  // --- brake pedal -----------------------------------------------------------
  brake_pedal_.enabled = declareBool(
    node_, "brake_pedal_feedback.enabled", true,
    "Decode brake pedal state from OBD frames forwarded by the CAN gateway.");
  brake_pedal_.can_id = static_cast<std::uint32_t>(declareInt(
    node_, "brake_pedal_feedback.can_id", 0x220, 0, 0x1FFFFFFF,
    "CAN identifier carrying the brake pedal signal. Default is the Kia Soul "
    "brake pressure ID -- set it for your own vehicle."));
  brake_pedal_.start_bit = static_cast<int>(declareInt(
    node_, "brake_pedal_feedback.start_bit", 0, 0, 63,
    "Position of the signal. Use bit_length=1 for a plain pedal switch."));
  brake_pedal_.bit_length = static_cast<int>(declareInt(
    node_, "brake_pedal_feedback.bit_length", 1, 1, 64,
    "Signal width. 1 means a boolean switch; wider means a pressure value "
    "compared against press_threshold."));
  brake_pedal_.little_endian = declareBool(
    node_, "brake_pedal_feedback.little_endian", true,
    "True for Intel byte order, false for Motorola.");
  brake_pedal_.is_signed = declareBool(
    node_, "brake_pedal_feedback.is_signed", false,
    "Interpret the raw value as two's complement.");
  brake_pedal_.scale = declareDouble(
    node_, "brake_pedal_feedback.scale", 1.0, -1000.0, 1000.0,
    "Multiplier applied to the raw value.");
  brake_pedal_.offset = declareDouble(
    node_, "brake_pedal_feedback.offset", 0.0, -100000.0, 100000.0,
    "Added after scaling.");

  brake_press_threshold_ = declareDouble(
    node_, "brake_pedal_feedback.press_threshold", 0.5, -100000.0, 100000.0,
    "Scaled value at or above which the pedal counts as pressed.");
  brake_active_high_ = declareBool(
    node_, "brake_pedal_feedback.active_high", true,
    "False inverts the pressed/released sense.");

  feedback_timeout_ = declareDouble(
    node_, "feedback_timeout", 1.0, 0.05, 60.0,
    "Seconds without a matching frame before a staleness warning is logged.");

  const rclcpp::QoS qos(10);
  steering_pub_ = node_->create_publisher<msg::SteeringAngleReport>("steering_angle", qos);
  brake_pub_ = node_->create_publisher<msg::BrakePedalReport>("brake_pedal", qos);

  if (steering_.enabled) {
    if (!steering_.fitsIn(8)) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "steering_feedback signal does not fit in an 8-byte frame (%s) -- disabled.",
        steering_.summary().c_str());
      steering_.enabled = false;
    } else {
      RCLCPP_INFO(node_->get_logger(), "Steering feedback: %s", steering_.summary().c_str());
    }
  }

  if (brake_pedal_.enabled) {
    if (!brake_pedal_.fitsIn(8)) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "brake_pedal_feedback signal does not fit in an 8-byte frame (%s) -- disabled.",
        brake_pedal_.summary().c_str());
      brake_pedal_.enabled = false;
    } else {
      RCLCPP_INFO(node_->get_logger(), "Brake pedal feedback: %s", brake_pedal_.summary().c_str());
    }
  }

  if (steering_min_ >= steering_max_) {
    RCLCPP_WARN(
      node_->get_logger(),
      "steering_feedback.min_angle >= max_angle; every sample will read in_range=false.");
  }
}

void ObdFeedback::process(const struct can_frame & frame, const rclcpp::Time & stamp)
{
  const std::uint32_t id = plainId(frame);

  if (steering_.enabled && id == steering_.can_id) {
    if (steering_.fitsIn(frame.can_dlc)) {
      msg::SteeringAngleReport report;
      report.header.stamp = stamp;
      report.can_id = id;
      report.raw = steering_.decodeRaw(frame.data, frame.can_dlc);
      report.angle = static_cast<double>(report.raw) * steering_.scale + steering_.offset;
      report.in_range = (report.angle >= steering_min_ && report.angle <= steering_max_);

      if (!report.in_range) {
        RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 2000,
          "Steering angle %.2f outside [%.1f, %.1f] -- check scale/offset/byte order.",
          report.angle, steering_min_, steering_max_);
      }

      steering_pub_->publish(report);
      last_steering_ = stamp;
    } else {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Frame 0x%X has dlc=%u, too short for the configured steering signal.",
        id, static_cast<unsigned>(frame.can_dlc));
    }
  }

  if (brake_pedal_.enabled && id == brake_pedal_.can_id) {
    if (brake_pedal_.fitsIn(frame.can_dlc)) {
      msg::BrakePedalReport report;
      report.header.stamp = stamp;
      report.can_id = id;
      report.raw = brake_pedal_.decodeRaw(frame.data, frame.can_dlc);
      report.value =
        static_cast<double>(report.raw) * brake_pedal_.scale + brake_pedal_.offset;

      const bool above = report.value >= brake_press_threshold_;
      report.pressed = brake_active_high_ ? above : !above;

      brake_pub_->publish(report);
      last_brake_ = stamp;
    } else {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "Frame 0x%X has dlc=%u, too short for the configured brake pedal signal.",
        id, static_cast<unsigned>(frame.can_dlc));
    }
  }
}

void ObdFeedback::checkStaleness(const rclcpp::Time & now)
{
  if (steering_.enabled && last_steering_.nanoseconds() > 0 &&
    (now - last_steering_).seconds() > feedback_timeout_)
  {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "No steering angle frame (0x%X) for over %.1f s.",
      steering_.can_id, feedback_timeout_);
  }

  if (brake_pedal_.enabled && last_brake_.nanoseconds() > 0 &&
    (now - last_brake_).seconds() > feedback_timeout_)
  {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "No brake pedal frame (0x%X) for over %.1f s.",
      brake_pedal_.can_id, feedback_timeout_);
  }
}

}  // namespace roscco
