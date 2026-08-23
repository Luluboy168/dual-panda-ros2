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

#include "franka_hardware/real/spsc_ring_buffer.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <initializer_list>
#include <thread>

namespace franka_hardware
{
namespace
{

constexpr std::uint64_t kHighVolumeValueCount = 250000;

TEST(SpscRingBufferTest, PreservesFifoOrderForFixedSizePayloads)
{
  using Payload = std::array<double, 3>;
  SpscRingBuffer<Payload, 3> queue;

  ASSERT_TRUE(queue.tryPush(Payload{1.0, 2.0, 3.0}));
  ASSERT_TRUE(queue.tryPush(Payload{4.0, 5.0, 6.0}));
  ASSERT_TRUE(queue.tryPush(Payload{7.0, 8.0, 9.0}));

  Payload value{};
  ASSERT_TRUE(queue.tryPop(value));
  EXPECT_EQ(value, (Payload{1.0, 2.0, 3.0}));
  ASSERT_TRUE(queue.tryPop(value));
  EXPECT_EQ(value, (Payload{4.0, 5.0, 6.0}));
  ASSERT_TRUE(queue.tryPop(value));
  EXPECT_EQ(value, (Payload{7.0, 8.0, 9.0}));
}

TEST(SpscRingBufferTest, ReportsEmptyAndFullWithoutChangingOutput)
{
  SpscRingBuffer<int, 2> queue;
  int output = 99;

  EXPECT_FALSE(queue.tryPop(output));
  EXPECT_EQ(output, 99);
  EXPECT_TRUE(queue.canPush());
  EXPECT_TRUE(queue.tryPush(10));
  EXPECT_TRUE(queue.tryPush(20));
  EXPECT_FALSE(queue.canPush());
  EXPECT_FALSE(queue.tryPush(30));

  ASSERT_TRUE(queue.tryPop(output));
  EXPECT_EQ(output, 10);
  EXPECT_TRUE(queue.canPush());
  EXPECT_TRUE(queue.tryPush(30));
  ASSERT_TRUE(queue.tryPop(output));
  EXPECT_EQ(output, 20);
  ASSERT_TRUE(queue.tryPop(output));
  EXPECT_EQ(output, 30);
  EXPECT_FALSE(queue.tryPop(output));
}

TEST(SpscRingBufferTest, PreservesOrderAcrossIndexWraparound)
{
  SpscRingBuffer<int, 3> queue;
  int output = 0;

  ASSERT_TRUE(queue.tryPush(1));
  ASSERT_TRUE(queue.tryPush(2));
  ASSERT_TRUE(queue.tryPush(3));
  ASSERT_TRUE(queue.tryPop(output));
  EXPECT_EQ(output, 1);
  ASSERT_TRUE(queue.tryPop(output));
  EXPECT_EQ(output, 2);

  ASSERT_TRUE(queue.tryPush(4));
  ASSERT_TRUE(queue.tryPush(5));
  EXPECT_FALSE(queue.tryPush(6));
  for (const int expected : {3, 4, 5}) {
    ASSERT_TRUE(queue.tryPop(output));
    EXPECT_EQ(output, expected);
  }
  EXPECT_FALSE(queue.tryPop(output));
}

TEST(SpscRingBufferTest, PopsNewestVisibleValueAndDiscardsOlderValues)
{
  SpscRingBuffer<int, 5> queue;
  int output = 0;

  ASSERT_TRUE(queue.tryPush(1));
  ASSERT_TRUE(queue.tryPush(2));
  ASSERT_TRUE(queue.tryPush(3));
  ASSERT_TRUE(queue.tryPush(4));
  ASSERT_TRUE(queue.popLatest(output));
  EXPECT_EQ(output, 4);
  EXPECT_FALSE(queue.tryPop(output));

  ASSERT_TRUE(queue.tryPush(5));
  ASSERT_TRUE(queue.tryPush(6));
  ASSERT_TRUE(queue.popLatest(output));
  EXPECT_EQ(output, 6);
  EXPECT_FALSE(queue.popLatest(output));
}

TEST(SpscRingBufferTest, ClearDiscardsValuesWhileQuiescent)
{
  SpscRingBuffer<int, 3> queue;
  int output = 0;

  ASSERT_TRUE(queue.tryPush(1));
  ASSERT_TRUE(queue.tryPush(2));
  queue.clear();
  EXPECT_FALSE(queue.tryPop(output));

  ASSERT_TRUE(queue.tryPush(3));
  ASSERT_TRUE(queue.tryPop(output));
  EXPECT_EQ(output, 3);
}

TEST(SpscRingBufferTest, TransfersHighVolumeSequenceBetweenTwoThreads)
{
  SpscRingBuffer<std::uint64_t, 64> queue;

  std::thread producer([&queue]() {
    for (std::uint64_t value = 1; value <= kHighVolumeValueCount; ++value) {
      while (!queue.tryPush(value)) {
        std::this_thread::yield();
      }
    }
  });

  bool order_preserved = true;
  std::uint64_t first_mismatch_expected = 0;
  std::uint64_t first_mismatch_actual = 0;
  for (std::uint64_t expected = 1; expected <= kHighVolumeValueCount; ++expected) {
    std::uint64_t actual = 0;
    while (!queue.tryPop(actual)) {
      std::this_thread::yield();
    }
    if (order_preserved && actual != expected) {
      order_preserved = false;
      first_mismatch_expected = expected;
      first_mismatch_actual = actual;
    }
  }
  producer.join();

  EXPECT_TRUE(order_preserved) << "first mismatch: expected " << first_mismatch_expected << ", got "
                               << first_mismatch_actual;
  std::uint64_t extra_value = 0;
  EXPECT_FALSE(queue.tryPop(extra_value));
}

}  // namespace
}  // namespace franka_hardware
