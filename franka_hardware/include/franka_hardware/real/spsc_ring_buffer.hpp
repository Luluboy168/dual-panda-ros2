// Copyright 2026 The multipanda_ros2 Authors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <type_traits>

namespace franka_hardware
{

/**
 * Fixed-capacity queue for one producer thread and one consumer thread.
 *
 * The queue does not allocate or lock in tryPush(), tryPop(), or popLatest().
 * T should be a fixed-size value whose copy assignment also
 * does not allocate when this class is used on a real-time path.
 *
 * Exactly one thread may call tryPush(). Exactly one other thread may call
 * tryPop() or popLatest(). clear() may be called only when both threads are
 * quiescent. Violating these ownership rules is undefined behavior.
 *
 * Capacity is the number of values the queue can contain. One additional
 * internal slot distinguishes a full queue from an empty queue.
 */
template <typename T, std::size_t Capacity>
class SpscRingBuffer
{
  static_assert(Capacity > 0, "SpscRingBuffer capacity must be greater than zero");
  static_assert(
    std::atomic<std::size_t>::is_always_lock_free,
    "SpscRingBuffer requires lock-free index atomics");
  static_assert(
    std::is_default_constructible_v<T>, "SpscRingBuffer values must be default constructible");
  static_assert(std::is_copy_assignable_v<T>, "SpscRingBuffer values must be copy assignable");

public:
  SpscRingBuffer() = default;
  SpscRingBuffer(const SpscRingBuffer &) = delete;
  SpscRingBuffer & operator=(const SpscRingBuffer &) = delete;
  SpscRingBuffer(SpscRingBuffer &&) = delete;
  SpscRingBuffer & operator=(SpscRingBuffer &&) = delete;

  static constexpr std::size_t capacity() noexcept { return Capacity; }

  /** Returns whether the producer can publish one value without blocking. */
  bool canPush() const noexcept
  {
    const auto write_index = write_index_.load(std::memory_order_relaxed);
    return increment(write_index) != read_index_.load(std::memory_order_acquire);
  }

  /**
   * Copies value into the queue, or returns false without modifying the queue
   * when it is full. Producer-thread only.
   */
  bool tryPush(const T & value) noexcept(std::is_nothrow_copy_assignable_v<T>)
  {
    const auto write_index = write_index_.load(std::memory_order_relaxed);
    const auto next_write_index = increment(write_index);

    // Acquire pairs with the consumer's release when it returns a slot.
    if (next_write_index == read_index_.load(std::memory_order_acquire)) {
      return false;
    }

    storage_[write_index] = value;
    // Publish the completed value to the consumer.
    write_index_.store(next_write_index, std::memory_order_release);
    return true;
  }

  /**
   * Copies the oldest queued value to value, or returns false without changing
   * value when the queue is empty. Consumer-thread only.
   */
  bool tryPop(T & value) noexcept(std::is_nothrow_copy_assignable_v<T>)
  {
    const auto read_index = read_index_.load(std::memory_order_relaxed);

    // Acquire pairs with the producer's release that published the value.
    if (read_index == write_index_.load(std::memory_order_acquire)) {
      return false;
    }

    value = storage_[read_index];
    // Return the consumed slot to the producer only after the copy completes.
    read_index_.store(increment(read_index), std::memory_order_release);
    return true;
  }

  /**
   * Copies the newest value visible at entry and discards all older visible
   * values. Values published concurrently after the write-index snapshot stay
   * queued for the next consumer call. Consumer-thread only.
   */
  bool popLatest(T & value) noexcept(std::is_nothrow_copy_assignable_v<T>)
  {
    const auto read_index = read_index_.load(std::memory_order_relaxed);
    const auto write_index = write_index_.load(std::memory_order_acquire);
    if (read_index == write_index) {
      return false;
    }

    const auto latest_index = write_index == 0 ? kStorageCapacity - 1 : write_index - 1;
    value = storage_[latest_index];
    // Return the entire observed range only after the newest value is copied.
    read_index_.store(write_index, std::memory_order_release);
    return true;
  }

  /** Clears all queued values. Call only while producer and consumer are quiescent. */
  void clear() noexcept
  {
    read_index_.store(write_index_.load(std::memory_order_relaxed), std::memory_order_relaxed);
  }

private:
  static constexpr std::size_t kStorageCapacity = Capacity + 1;
  static constexpr std::size_t kCacheLineSize = 64;

  static constexpr std::size_t increment(std::size_t index) noexcept
  {
    return index + 1 == kStorageCapacity ? 0 : index + 1;
  }

  std::array<T, kStorageCapacity> storage_{};

  // Separate cache lines avoid producer/consumer contention on the index
  // objects themselves. Each index has exactly one writer.
  alignas(kCacheLineSize) std::atomic<std::size_t> write_index_{0};
  alignas(kCacheLineSize) std::atomic<std::size_t> read_index_{0};
};

}  // namespace franka_hardware
