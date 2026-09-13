#include "roscco/oscc_to_ros.hpp"

#include <chrono>
#include <memory>

namespace roscco
{

SignalQueue<Stamped<oscc_brake_report_s>, 64> OsccToRos::brake_queue_;
SignalQueue<Stamped<oscc_steering_report_s>, 64> OsccToRos::steering_queue_;
SignalQueue<Stamped<oscc_throttle_report_s>, 64> OsccToRos::throttle_queue_;
SignalQueue<Stamped<oscc_fault_report_s>, 64> OsccToRos::fault_queue_;
SignalQueue<Stamped<struct can_frame>, 256> OsccToRos::obd_queue_;
std::atomic<std::uint64_t> OsccToRos::dropped_{0};

namespace
{

/// Enqueue a report with an async-signal-safe timestamp.
template <typename T, typename Q>
void enqueue(Q & queue, const T * report, std::atomic<std::uint64_t> & dropped)
{
  if (report == nullptr) {
    return;
  }

  Stamped<T> stamped;
  stamped.data = *report;
  clock_gettime(CLOCK_REALTIME, &stamped.stamp);

  if (!queue.push(stamped)) {
    dropped.fetch_add(1, std::memory_order_relaxed);
  }
}

rclcpp::Time toRosTime(const struct timespec & ts)
{
  return rclcpp::Time(
    static_cast<std::int32_t>(ts.tv_sec),
    static_cast<std::uint32_t>(ts.tv_nsec),
    RCL_ROS_TIME);
}

/**
 * @brief Copy an OSCC report into its ROS counterpart, field by field.
 *
 * The ROS 1 version reinterpret_cast the OSCC struct straight onto the
 * generated message struct. That was already fragile; in ROS 2 the generated
 * types hold std::array members and an allocator, so the layouts genuinely do
 * not match and the cast is undefined behaviour. Explicit copies instead.
 */
template <typename RosData, typename OsccReport>
void fillReportData(RosData & out, const OsccReport & in)
{
  out.magic[0] = in.magic[0];
  out.magic[1] = in.magic[1];
  out.enabled = in.enabled;
  out.operator_override = in.operator_override;
  out.dtcs = in.dtcs;
  out.reserved[0] = in.reserved[0];
  out.reserved[1] = in.reserved[1];
  out.reserved[2] = in.reserved[2];
}

}  // namespace

OsccToRos::OsccToRos(rclcpp::Node * node, int drain_period_ms)
: node_(node)
{
  sigset_t mask;
  sigset_t orig_mask;

  sigemptyset(&mask);
  sigemptyset(&orig_mask);
  sigaddset(&mask, SIGIO);

  // Block OSCC's SIGIO while the publishers are being built, so a report
  // cannot land mid-construction.
  if (sigprocmask(SIG_BLOCK, &mask, &orig_mask) < 0) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to block SIGIO");
  }

  const rclcpp::QoS qos(10);

  brake_pub_ = node_->create_publisher<msg::BrakeReport>("brake_report", qos);
  steering_pub_ = node_->create_publisher<msg::SteeringReport>("steering_report", qos);
  throttle_pub_ = node_->create_publisher<msg::ThrottleReport>("throttle_report", qos);
  fault_pub_ = node_->create_publisher<msg::FaultReport>("fault_report", qos);
  obd_pub_ = node_->create_publisher<msg::CanFrame>("can_frame", qos);

  if (sigprocmask(SIG_SETMASK, &orig_mask, nullptr) < 0) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to unblock SIGIO");
  }

  feedback_ = std::make_unique<ObdFeedback>(node_);

  drain_timer_ = node_->create_wall_timer(
    std::chrono::milliseconds(drain_period_ms),
    std::bind(&OsccToRos::drain, this));

  oscc_subscribe_to_brake_reports(brakeCallback);
  oscc_subscribe_to_steering_reports(steeringCallback);
  oscc_subscribe_to_throttle_reports(throttleCallback);
  oscc_subscribe_to_fault_reports(faultCallback);
  oscc_subscribe_to_obd_messages(obdCallback);
}

OsccToRos::~OsccToRos()
{
  // Detach before the publishers die, or a late SIGIO writes into freed queues.
  oscc_subscribe_to_brake_reports(nullptr);
  oscc_subscribe_to_steering_reports(nullptr);
  oscc_subscribe_to_throttle_reports(nullptr);
  oscc_subscribe_to_fault_reports(nullptr);
  oscc_subscribe_to_obd_messages(nullptr);
}

void OsccToRos::brakeCallback(oscc_brake_report_s * report)
{
  enqueue(brake_queue_, report, dropped_);
}

void OsccToRos::steeringCallback(oscc_steering_report_s * report)
{
  enqueue(steering_queue_, report, dropped_);
}

void OsccToRos::throttleCallback(oscc_throttle_report_s * report)
{
  enqueue(throttle_queue_, report, dropped_);
}

void OsccToRos::faultCallback(oscc_fault_report_s * report)
{
  enqueue(fault_queue_, report, dropped_);
}

void OsccToRos::obdCallback(struct can_frame * frame)
{
  enqueue(obd_queue_, frame, dropped_);
}

void OsccToRos::drain()
{
  {
    Stamped<oscc_brake_report_s> item;
    while (brake_queue_.pop(item)) {
      msg::BrakeReport out;
      out.header.stamp = toRosTime(item.stamp);
      fillReportData(out.data, item.data);
      brake_pub_->publish(out);
    }
  }

  {
    Stamped<oscc_steering_report_s> item;
    while (steering_queue_.pop(item)) {
      msg::SteeringReport out;
      out.header.stamp = toRosTime(item.stamp);
      fillReportData(out.data, item.data);
      steering_pub_->publish(out);
    }
  }

  {
    Stamped<oscc_throttle_report_s> item;
    while (throttle_queue_.pop(item)) {
      msg::ThrottleReport out;
      out.header.stamp = toRosTime(item.stamp);
      fillReportData(out.data, item.data);
      throttle_pub_->publish(out);
    }
  }

  {
    Stamped<oscc_fault_report_s> item;
    while (fault_queue_.pop(item)) {
      msg::FaultReport out;
      out.header.stamp = toRosTime(item.stamp);
      out.data.magic[0] = item.data.magic[0];
      out.data.magic[1] = item.data.magic[1];
      out.data.fault_origin_id = item.data.fault_origin_id;
      out.data.dtcs = item.data.dtcs;
      out.data.reserved = item.data.reserved;
      fault_pub_->publish(out);

      RCLCPP_ERROR(
        node_->get_logger(),
        "OSCC fault report: origin=%u dtcs=0x%02X",
        static_cast<unsigned>(item.data.fault_origin_id),
        static_cast<unsigned>(item.data.dtcs));
    }
  }

  {
    Stamped<struct can_frame> item;
    while (obd_queue_.pop(item)) {
      msg::CanFrame out;
      out.header.stamp = toRosTime(item.stamp);
      out.frame.can_id = item.data.can_id;
      out.frame.can_dlc = item.data.can_dlc;
      for (std::size_t i = 0; i < out.frame.data.size(); ++i) {
        out.frame.data[i] = item.data.data[i];
      }
      obd_pub_->publish(out);

      // Raw frame goes out regardless; the decoder picks out the IDs it wants.
      feedback_->process(item.data, out.header.stamp);
    }
  }

  feedback_->checkStaleness(node_->get_clock()->now());

  const std::uint64_t drops = dropped_.load(std::memory_order_relaxed);
  if (drops != last_reported_drops_) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "Dropped %lu OSCC report(s): queue full, executor is not keeping up",
      static_cast<unsigned long>(drops));
    last_reported_drops_ = drops;
  }
}

}  // namespace roscco
