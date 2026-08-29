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
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace franka_ik::test {

inline std::string readTextFile(const std::string& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    throw std::runtime_error("cannot read test file '" + path + "'");
  }
  return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

inline std::uint32_t rotateRight(const std::uint32_t value, const unsigned int count) noexcept {
  return (value >> count) | (value << (32U - count));
}

// Independent test-only SHA-256 implementation. Production deliberately uses rcutils instead.
inline std::string sha256Hex(const std::string_view input) {
  // clang-format off
  constexpr std::array<std::uint32_t, 64> kRoundConstants{{
      0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U, 0x3956c25bU, 0x59f111f1U,
      0x923f82a4U, 0xab1c5ed5U, 0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U,
      0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U, 0xe49b69c1U, 0xefbe4786U,
      0x0fc19dc6U, 0x240ca1ccU, 0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU,
      0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U, 0xc6e00bf3U, 0xd5a79147U,
      0x06ca6351U, 0x14292967U, 0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U,
      0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U, 0xa2bfe8a1U, 0xa81a664bU,
      0xc24b8b70U, 0xc76c51a3U, 0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U,
      0x19a4c116U, 0x1e376c08U, 0x2748774cU, 0x34b0bcb5U, 0x391c0cb3U, 0x4ed8aa4aU,
      0x5b9cca4fU, 0x682e6ff3U, 0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
      0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U,
  }};
  std::array<std::uint32_t, 8> hash{{0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U,
                                      0xa54ff53aU, 0x510e527fU, 0x9b05688cU,
                                      0x1f83d9abU, 0x5be0cd19U}};
  // clang-format on

  std::vector<std::uint8_t> padded(input.begin(), input.end());
  const std::uint64_t input_bits = static_cast<std::uint64_t>(padded.size()) * 8U;
  padded.push_back(0x80U);
  while ((padded.size() % 64U) != 56U) {
    padded.push_back(0U);
  }
  for (int shift = 56; shift >= 0; shift -= 8) {
    padded.push_back(static_cast<std::uint8_t>(input_bits >> static_cast<unsigned int>(shift)));
  }

  for (std::size_t block = 0; block < padded.size(); block += 64U) {
    std::array<std::uint32_t, 64> words{};
    for (std::size_t index = 0; index < 16U; ++index) {
      const std::size_t offset = block + 4U * index;
      words[index] = (static_cast<std::uint32_t>(padded[offset]) << 24U) |
                     (static_cast<std::uint32_t>(padded[offset + 1U]) << 16U) |
                     (static_cast<std::uint32_t>(padded[offset + 2U]) << 8U) |
                     static_cast<std::uint32_t>(padded[offset + 3U]);
    }
    for (std::size_t index = 16U; index < words.size(); ++index) {
      const std::uint32_t sigma0 = rotateRight(words[index - 15U], 7U) ^
                                   rotateRight(words[index - 15U], 18U) ^
                                   (words[index - 15U] >> 3U);
      const std::uint32_t sigma1 = rotateRight(words[index - 2U], 17U) ^
                                   rotateRight(words[index - 2U], 19U) ^ (words[index - 2U] >> 10U);
      words[index] = words[index - 16U] + sigma0 + words[index - 7U] + sigma1;
    }

    std::uint32_t a = hash[0];
    std::uint32_t b = hash[1];
    std::uint32_t c = hash[2];
    std::uint32_t d = hash[3];
    std::uint32_t e = hash[4];
    std::uint32_t f = hash[5];
    std::uint32_t g = hash[6];
    std::uint32_t h = hash[7];
    for (std::size_t index = 0; index < words.size(); ++index) {
      const std::uint32_t sum1 = rotateRight(e, 6U) ^ rotateRight(e, 11U) ^ rotateRight(e, 25U);
      const std::uint32_t choice = (e & f) ^ ((~e) & g);
      const std::uint32_t temporary1 = h + sum1 + choice + kRoundConstants[index] + words[index];
      const std::uint32_t sum0 = rotateRight(a, 2U) ^ rotateRight(a, 13U) ^ rotateRight(a, 22U);
      const std::uint32_t majority = (a & b) ^ (a & c) ^ (b & c);
      const std::uint32_t temporary2 = sum0 + majority;
      h = g;
      g = f;
      f = e;
      e = d + temporary1;
      d = c;
      c = b;
      b = a;
      a = temporary1 + temporary2;
    }
    hash[0] += a;
    hash[1] += b;
    hash[2] += c;
    hash[3] += d;
    hash[4] += e;
    hash[5] += f;
    hash[6] += g;
    hash[7] += h;
  }

  std::ostringstream result;
  result << std::hex << std::setfill('0');
  for (const std::uint32_t value : hash) {
    result << std::setw(8) << value;
  }
  return result.str();
}

}  // namespace franka_ik::test
