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
#include <cstring>
#include <deque>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <thread>

namespace franka_hardware {
namespace {

constexpr std::uint64_t kHighVolumeValueCount = 250000;

TEST(SpscRingBufferTest, PreservesFifoOrderForFixedSizePayloads) {
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

TEST(SpscRingBufferTest, ReportsEmptyAndFullWithoutChangingOutput) {
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

TEST(SpscRingBufferTest, PreservesOrderAcrossIndexWraparound) {
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

TEST(SpscRingBufferTest, PopsNewestVisibleValueAndDiscardsOlderValues) {
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

TEST(SpscRingBufferTest, ClearDiscardsValuesWhileQuiescent) {
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

TEST(SpscRingBufferTest, TransfersHighVolumeSequenceBetweenTwoThreads) {
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

// ---------------------------------------------------------------------------
// Fixed-seed boundary/fuzz test: randomized push/pop/popLatest/clear
// sequences, single-threaded, checked against an independent std::deque
// reference model across several capacities including the tightest boundary
// (Capacity == 1). Payloads are hostile doubles (NaN, +-Inf, subnormals,
// signaling-NaN bit patterns, +-0.0) compared bit-exactly so NaN payloads
// cannot silently make the comparison vacuously pass via `NaN != NaN`.
// ---------------------------------------------------------------------------

bool sameBits(double lhs, double rhs) {
  std::uint64_t lhs_bits = 0;
  std::uint64_t rhs_bits = 0;
  std::memcpy(&lhs_bits, &lhs, sizeof(lhs_bits));
  std::memcpy(&rhs_bits, &rhs, sizeof(rhs_bits));
  return lhs_bits == rhs_bits;
}

double randomHostileDouble(std::mt19937_64& engine) {
  switch (engine() % 10U) {
    case 0:
      return std::numeric_limits<double>::quiet_NaN();
    case 1:
      return std::numeric_limits<double>::infinity();
    case 2:
      return -std::numeric_limits<double>::infinity();
    case 3:
      return 0.0;
    case 4:
      return -0.0;
    case 5:
      return std::numeric_limits<double>::denorm_min();
    case 6:
      return std::numeric_limits<double>::max();
    case 7:
      return std::numeric_limits<double>::lowest();
    case 8: {
      std::uniform_real_distribution<double> dist(-1e6, 1e6);
      return dist(engine);
    }
    default: {
      // A signaling-NaN-shaped bit pattern (quiet bit clear, nonzero mantissa).
      std::uint64_t bits = 0x7ff0000000000001ULL | (engine() & 0xFULL);
      double value = 0.0;
      std::memcpy(&value, &bits, sizeof(value));
      return value;
    }
  }
}

template <std::size_t Capacity>
std::size_t runRingBufferFuzz(std::uint64_t seed, std::size_t operation_count) {
  std::cout << "SpscRingBuffer property capacity=" << Capacity << " seed=" << seed
            << " ops=" << operation_count << '\n';
  SpscRingBuffer<double, Capacity> queue;
  std::deque<double> model;
  std::mt19937_64 engine(seed);

  for (std::size_t op = 0; op < operation_count; ++op) {
    const auto action = engine() % 10U;
    SCOPED_TRACE("capacity=" + std::to_string(Capacity) + " seed=" + std::to_string(seed) +
                 " op=" + std::to_string(op) + " action=" + std::to_string(action) +
                 " model_size=" + std::to_string(model.size()));

    if (action <= 5U) {  // push-heavy bias to reliably hit the full-queue boundary
      const double value = randomHostileDouble(engine);
      const bool model_can_push = model.size() < Capacity;
      EXPECT_EQ(queue.canPush(), model_can_push);
      const bool pushed = queue.tryPush(value);
      EXPECT_EQ(pushed, model_can_push);
      if (pushed) {
        model.push_back(value);
      }
    } else if (action <= 7U) {  // pop oldest (FIFO)
      double output = 0.0;
      const bool model_has_value = !model.empty();
      const bool popped = queue.tryPop(output);
      EXPECT_EQ(popped, model_has_value);
      if (popped) {
        EXPECT_TRUE(sameBits(output, model.front()));
        model.pop_front();
      }
    } else if (action == 8U) {  // popLatest: newest value, discard all older
      double output = 0.0;
      const bool model_has_value = !model.empty();
      const bool popped = queue.popLatest(output);
      EXPECT_EQ(popped, model_has_value);
      if (popped) {
        EXPECT_TRUE(sameBits(output, model.back()));
        model.clear();
      }
    } else {  // clear: valid single-threaded since producer/consumer are one thread here
      queue.clear();
      model.clear();
    }
  }

  // Drain and require the exact remaining FIFO order matches the model.
  double output = 0.0;
  std::size_t drained = 0;
  while (queue.tryPop(output)) {
    EXPECT_FALSE(model.empty());
    EXPECT_TRUE(sameBits(output, model.front()));
    model.pop_front();
    ++drained;
  }
  EXPECT_TRUE(model.empty());
  return operation_count;
}

TEST(SpscRingBufferTest, FixedSeedBoundaryFuzzMatchesReferenceFifoModelAcrossCapacities) {
  constexpr std::array<std::uint64_t, 3> kSeeds{0x53505343U, 0xb0110cd0U, 0x72696e6762756631ULL};
  constexpr std::array<std::size_t, 5> kCapacities{1, 2, 3, 7, 64};
  constexpr std::size_t kOperationsPerCase = 3000;
  std::size_t total_cases = 0;

  for (const auto seed : kSeeds) {
    total_cases += runRingBufferFuzz<1>(seed, kOperationsPerCase);
    total_cases += runRingBufferFuzz<2>(seed, kOperationsPerCase);
    total_cases += runRingBufferFuzz<3>(seed, kOperationsPerCase);
    total_cases += runRingBufferFuzz<7>(seed, kOperationsPerCase);
    total_cases += runRingBufferFuzz<64>(seed, kOperationsPerCase);
  }

  std::cout << "SpscRingBuffer property total generated cases=" << total_cases
            << " (seeds=" << kSeeds.size() << " capacities=" << kCapacities.size()
            << " ops-per-case=" << kOperationsPerCase << ")\n";
  EXPECT_EQ(total_cases, kSeeds.size() * kCapacities.size() * kOperationsPerCase);
}

TEST(SpscRingBufferTest, CapacityOneAdmitsExactlyOneOutstandingValueAtATime) {
  SpscRingBuffer<double, 1> queue;
  EXPECT_EQ(queue.capacity(), 1U);
  EXPECT_TRUE(queue.canPush());

  const double nan_value = std::numeric_limits<double>::quiet_NaN();
  ASSERT_TRUE(queue.tryPush(nan_value));
  EXPECT_FALSE(queue.canPush());
  EXPECT_FALSE(queue.tryPush(1.0));  // must not overwrite the pending value

  double output = 0.0;
  ASSERT_TRUE(queue.tryPop(output));
  EXPECT_TRUE(sameBits(output, nan_value));
  EXPECT_FALSE(queue.tryPop(output));
  EXPECT_TRUE(queue.canPush());
}

}  // namespace
}  // namespace franka_hardware
