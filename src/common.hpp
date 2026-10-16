// SPDX-License-Identifier: BSD-3-Clause
/** @file common.hpp
 * @brief UAV names, elapsed-time measurement, and message framing.
 */
#pragma once

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace mrs_uav_wifi_direct {
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

/** @brief Form a UAV name by appending its number to "uav". */
inline std::string uav_name(int number) {
	return "uav" + std::to_string(number);
}

/** @brief Append a 1-, 2-, 4- or 8-byte integer in network byte order. */
inline void append(std::vector<uint8_t>& bytes, uint64_t value, size_t width) {
	for (size_t i = width; i > 0; --i) {
		bytes.push_back(static_cast<uint8_t>(value >> ((i - 1) * 8)));
	}
}
/** @brief Decode an integer after the caller checks that width bytes are available. */
inline uint64_t integer(const uint8_t* bytes, size_t width) {
	uint64_t value{};
	for (size_t i = 0; i < width; ++i) {
		value = (value << 8) | bytes[i];
	}
	return value;
}

/**
 * @brief Message buffer passed from a MsQuic worker to the ROS executor.
 * The boot instance and topic sequence distinguish reconnects and discard late or
 * duplicate samples when the sender changes between AP and direct connections.
 * Path identifies the matching locally configured hostname-scoped topic.
 */
struct Received {
	int peer{};
	uint64_t instance{}, sequence{};
	std::string path;
	std::vector<uint8_t> payload;
};

}  // namespace mrs_uav_wifi_direct
