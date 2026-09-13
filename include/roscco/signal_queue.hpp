#ifndef ROSCCO__SIGNAL_QUEUE_HPP_
#define ROSCCO__SIGNAL_QUEUE_HPP_

#include <array>
#include <atomic>
#include <cstddef>
#include <ctime>

namespace roscco
{

/**
 * @brief A POD payload plus the wall-clock instant it arrived.
 *
 * clock_gettime() is async-signal-safe, so the timestamp is taken inside the
 * OSCC callback rather than later in the drain timer. That keeps the stamp
 * close to the actual CAN frame instead of reflecting scheduler jitter.
 */
template <typename T>
struct Stamped
{
  T data{};
  struct timespec stamp
  {
    0, 0
  };
};

/**
 * @brief Single-producer / single-consumer lock-free ring buffer.
 *
 * The producer is the OSCC SIGIO handler; the consumer is a regular rclcpp
 * timer callback. Only trivially-copyable payloads are allowed, so push() and
 * pop() do nothing but a struct assignment and two atomic operations -- both
 * async-signal-safe on any platform where std::atomic<size_t> is lock-free.
 *
 * This exists because publishing directly from a signal handler (as the ROS 1
 * version did) is not safe: rclcpp publishers allocate and take locks.
 */
template <typename T, std::size_t N>
class SignalQueue
{
  static_assert(N >= 2, "SignalQueue needs at least two slots");

public:
  /// Called from the signal handler. Returns false if the queue is full.
  bool push(const T & value) noexcept
  {
    const std::size_t head = head_.load(std::memory_order_relaxed);
    const std::size_t next = (head + 1) % N;

    if (next == tail_.load(std::memory_order_acquire)) {
      return false;  // full; drop the oldest-arriving sample rather than block
    }

    buffer_[head] = value;
    head_.store(next, std::memory_order_release);
    return true;
  }

  /// Called from the drain timer. Returns false if the queue is empty.
  bool pop(T & out) noexcept
  {
    const std::size_t tail = tail_.load(std::memory_order_relaxed);

    if (tail == head_.load(std::memory_order_acquire)) {
      return false;
    }

    out = buffer_[tail];
    tail_.store((tail + 1) % N, std::memory_order_release);
    return true;
  }

private:
  std::array<T, N> buffer_{};
  std::atomic<std::size_t> head_{0};
  std::atomic<std::size_t> tail_{0};
};

}  // namespace roscco

#endif  // ROSCCO__SIGNAL_QUEUE_HPP_
