#ifndef ROSCCO__PARAMETER_UTILS_HPP_
#define ROSCCO__PARAMETER_UTILS_HPP_

#include <cstdint>
#include <string>

#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>

namespace roscco
{

/**
 * @brief Declare a double parameter constrained to [lo, hi].
 *
 * The range lives in the descriptor, so rclcpp rejects out-of-range values at
 * declaration time and on every later `ros2 param set`. That matters more here
 * than in a typical node: a typo in a limit is a typo in how hard the car
 * brakes.
 */
inline double declareDouble(
  rclcpp::Node * node, const std::string & name, double default_value,
  double lo, double hi, const std::string & description, double step = 0.0)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;

  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = lo;
  range.to_value = hi;
  range.step = step;
  descriptor.floating_point_range.push_back(range);

  return node->declare_parameter<double>(name, default_value, descriptor);
}

/// Declare an integer parameter constrained to [lo, hi].
inline std::int64_t declareInt(
  rclcpp::Node * node, const std::string & name, std::int64_t default_value,
  std::int64_t lo, std::int64_t hi, const std::string & description, std::int64_t step = 1)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;

  rcl_interfaces::msg::IntegerRange range;
  range.from_value = lo;
  range.to_value = hi;
  range.step = step;
  descriptor.integer_range.push_back(range);

  return node->declare_parameter<std::int64_t>(name, default_value, descriptor);
}

/// Declare a bool parameter with a description (booleans take no range).
inline bool declareBool(
  rclcpp::Node * node, const std::string & name, bool default_value,
  const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = description;
  return node->declare_parameter<bool>(name, default_value, descriptor);
}

}  // namespace roscco

#endif  // ROSCCO__PARAMETER_UTILS_HPP_
