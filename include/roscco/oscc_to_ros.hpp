#ifndef ROSCCO__OSCC_TO_ROS_HPP_
#define ROSCCO__OSCC_TO_ROS_HPP_

#include <atomic>
#include <csignal>
#include <cstdint>

extern "C" {
#include <oscc.h>
}

#include <rclcpp/rclcpp.hpp>

#include "roscco/msg/brake_report.hpp"
#include "roscco/msg/can_frame.hpp"
#include "roscco/msg/fault_report.hpp"
#include "roscco/msg/steering_report.hpp"
#include "roscco/msg/throttle_report.hpp"
#include "roscco/signal_queue.hpp"

namespace roscco
{

/**
 * @brief Publishes OSCC reports onto ROS 2 topics.
 *
 * OSCC delivers reports from a SIGIO handler. Those handlers only enqueue into
 * lock-free ring buffers here; a timer owned by the node drains them and does
 * the actual publishing on the executor thread.
 */
class OsccToRos
{
public:
  /**
   * @param node             Node used for publishers, the drain timer and logging.
   * @param drain_period_ms  How often queued reports are published. Reports
   *                         arrive at 50 Hz per module, so anything at or below
   *                         10 ms adds negligible latency.
   */
  explicit OsccToRos(rclcpp::Node * node, int drain_period_ms = 5);

  ~OsccToRos();

  OsccToRos(const OsccToRos &) = delete;
  OsccToRos & operator=(const OsccToRos &) = delete;

private:
  // --- OSCC callbacks. Signal-handler context: enqueue only, never publish. ---
  static void brakeCallback(oscc_brake_report_s * report);
  static void steeringCallback(oscc_steering_report_s * report);
  static void throttleCallback(oscc_throttle_report_s * report);
  static void faultCallback(oscc_fault_report_s * report);
  static void obdCallback(struct can_frame * frame);

  /// Timer callback: moves everything queued onto ROS topics.
  void drain();

  rclcpp::Node * node_;

  rclcpp::Publisher<msg::BrakeReport>::SharedPtr brake_pub_;
  rclcpp::Publisher<msg::SteeringReport>::SharedPtr steering_pub_;
  rclcpp::Publisher<msg::ThrottleReport>::SharedPtr throttle_pub_;
  rclcpp::Publisher<msg::FaultReport>::SharedPtr fault_pub_;
  rclcpp::Publisher<msg::CanFrame>::SharedPtr obd_pub_;

  rclcpp::TimerBase::SharedPtr drain_timer_;

  // Depth 64 at 50 Hz per module is over a second of slack.
  static SignalQueue<Stamped<oscc_brake_report_s>, 64> brake_queue_;
  static SignalQueue<Stamped<oscc_steering_report_s>, 64> steering_queue_;
  static SignalQueue<Stamped<oscc_throttle_report_s>, 64> throttle_queue_;
  static SignalQueue<Stamped<oscc_fault_report_s>, 64> fault_queue_;
  static SignalQueue<Stamped<struct can_frame>, 256> obd_queue_;

  /// Incremented in the signal handler when a queue is full.
  static std::atomic<std::uint64_t> dropped_;
  std::uint64_t last_reported_drops_{0};
};

}  // namespace roscco

#endif  // ROSCCO__OSCC_TO_ROS_HPP_
