// SPDX-License-Identifier: BSD-3-Clause
/** @file radio_metrics.hpp
 * @brief Read nl80211 interface state and existing station statistics.
 */
#pragma once

#include <linux/nl80211.h>
#include <netlink/attr.h>
#include <netlink/genl/ctrl.h>
#include <netlink/genl/genl.h>

#include <cstdio>
#include <exception>
#include <functional>
#include <limits>
#include <memory>
#include <mrs_uav_wifi_direct/msg/link_metrics.hpp>

#include "common.hpp"
#include "network_utils.hpp"

namespace mrs_uav_wifi_direct {
/** @brief Initialize unknown floating-point measurements and counters explicitly. */
inline msg::LinkMetrics empty_link() {
	msg::LinkMetrics result;
	const auto nan = std::numeric_limits<double>::quiet_NaN();
	result.signal_dbm = result.signal_average_dbm = nan;
	result.tx_bitrate_mbps = result.rx_bitrate_mbps = result.expected_throughput_mbps = nan;
	result.inactive_s = result.rtt_ms = result.minimum_rtt_ms = result.maximum_rtt_ms = nan;
	result.rtt_variation_ms = result.rtt_age_s = nan;
	result.tx_packets = result.rx_packets = result.tx_retries = result.tx_failed = result.rx_dropped = UINT64_MAX;
	return result;
}
/**
 * @brief One bounded, read-only generic-netlink request with automatic resource cleanup.
 * Each request owns its socket and callbacks. Kernel errors, missing permissions and
 * timeouts become exceptions handled by the telemetry worker. Callbacks retain any
 * C++ exception until libnl returns, so none escapes through a C stack frame.
 */
inline void radio_query(
	int operation, int attribute, uint32_t value, bool dump, const std::function<void(nlattr**)>& inspect) {
	using Socket = std::unique_ptr<nl_sock, decltype(&nl_socket_free)>;
	using Message = std::unique_ptr<nl_msg, decltype(&nlmsg_free)>;
	Socket socket(nl_socket_alloc(), nl_socket_free);
	if (!socket || genl_connect(socket.get()) < 0) {
		throw std::runtime_error("Cannot open nl80211 socket");
	}
	timeval timeout{1, 0};
	if (::setsockopt(nl_socket_get_fd(socket.get()), SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
		throw std::runtime_error("Cannot set nl80211 timeout");
	}
	const auto family = genl_ctrl_resolve(socket.get(), "nl80211");
	if (family < 0) {
		throw std::runtime_error("nl80211 is unavailable");
	}
	Message message(nlmsg_alloc(), nlmsg_free);
	if (!message ||
		!genlmsg_put(message.get(), NL_AUTO_PORT, NL_AUTO_SEQ, family, 0, dump ? NLM_F_DUMP : 0, operation, 0) ||
		nla_put_u32(message.get(), attribute, value) < 0) {
		throw std::runtime_error("Cannot allocate nl80211 request");
	}
	struct Reply {
		const std::function<void(nlattr**)>& inspect;
		bool done{};
		int error{};
		std::exception_ptr exception;
	} reply{inspect, false, 0, {}};
	auto finish = [](nl_msg*, void* data) -> int {
		static_cast<Reply*>(data)->done = true;
		return NL_STOP;
	};
	nl_socket_modify_cb(socket.get(), dump ? NL_CB_FINISH : NL_CB_ACK, NL_CB_CUSTOM, finish, &reply);
	nl_socket_modify_err_cb(
		socket.get(), NL_CB_CUSTOM,
		[](sockaddr_nl*, nlmsgerr* error, void* data) -> int {
			auto& result = *static_cast<Reply*>(data);
			result.error = error->error;
			result.done = true;
			return NL_STOP;
		},
		&reply);
	nl_socket_modify_cb(
		socket.get(), NL_CB_VALID, NL_CB_CUSTOM,
		[](nl_msg* message, void* data) -> int {
			auto& result = *static_cast<Reply*>(data);
			nlattr* attrs[NL80211_ATTR_MAX + 1]{};
			if (genlmsg_parse(nlmsg_hdr(message), 0, attrs, NL80211_ATTR_MAX, nullptr) < 0) {
				return NL_SKIP;
			}
			try {
				result.inspect(attrs);
			} catch (...) {
				result.exception = std::current_exception();
				result.done = true;
				return NL_STOP;
			}
			return NL_OK;
		},
		&reply);
	int error = nl_send_auto(socket.get(), message.get());
	const auto deadline = Clock::now() + 2s;
	while (error >= 0 && !reply.done && Clock::now() < deadline) {
		error = nl_recvmsgs_default(socket.get());
	}
	if (reply.exception) {
		std::rethrow_exception(reply.exception);
	}
	if (reply.error || error < 0 || !reply.done) {
		throw std::runtime_error("nl80211 query failed: " +
			(reply.error		? std::string(std::strerror(-reply.error))
					: error < 0 ? nl_geterror(error)
								: "timeout"));
	}
}
/** @brief Format one kernel-provided MAC address in the same form as supplicant records. */
inline std::string radio_mac(nlattr* attribute) {
	if (!attribute || nla_len(attribute) != 6) {
		return {};
	}
	const auto* bytes = static_cast<const uint8_t*>(nla_data(attribute));
	char text[18];
	std::snprintf(text, sizeof(text), "%02x:%02x:%02x:%02x:%02x:%02x", bytes[0], bytes[1], bytes[2], bytes[3], bytes[4],
		bytes[5]);
	return text;
}
/** @brief Read a negotiated station bitrate, converting the kernel's 100 kbit/s units to Mbit/s. */
inline double station_bitrate(nlattr* attribute) {
	nlattr* rate[NL80211_RATE_INFO_MAX + 1]{};
	if (attribute && nla_parse_nested(rate, NL80211_RATE_INFO_MAX, attribute, nullptr) == 0) {
		if (rate[NL80211_RATE_INFO_BITRATE32]) {
			return nla_get_u32(rate[NL80211_RATE_INFO_BITRATE32]) / 10.0;
		}
		if (rate[NL80211_RATE_INFO_BITRATE]) {
			return nla_get_u16(rate[NL80211_RATE_INFO_BITRATE]) / 10.0;
		}
	}
	return std::numeric_limits<double>::quiet_NaN();
}
/**
 * @brief Read existing associated stations on an interface, without transmitting probes.
 * A managed interface normally returns its AP; a group owner returns its clients.
 * Absent driver fields retain explicit unknown values rather than implying zero loss.
 */
inline std::vector<msg::LinkMetrics> station_metrics(const std::string& interface) {
	std::vector<msg::LinkMetrics> result;
	const auto index = ::if_nametoindex(interface.c_str());
	if (!index) {
		throw std::runtime_error("Wi-Fi interface is unavailable: " + interface);
	}
	radio_query(NL80211_CMD_GET_STATION, NL80211_ATTR_IFINDEX, index, true, [&](nlattr** attrs) {
		if (!attrs[NL80211_ATTR_STA_INFO]) {
			return;
		}
		nlattr* station[NL80211_STA_INFO_MAX + 1]{};
		if (nla_parse_nested(station, NL80211_STA_INFO_MAX, attrs[NL80211_ATTR_STA_INFO], nullptr) < 0) {
			return;
		}
		auto link = empty_link();
		link.interface = interface;
		link.mac_address = radio_mac(attrs[NL80211_ATTR_MAC]);
		link.connected = true;
		if (station[NL80211_STA_INFO_SIGNAL]) {
			link.signal_dbm = static_cast<int8_t>(nla_get_u8(station[NL80211_STA_INFO_SIGNAL]));
		}
		if (station[NL80211_STA_INFO_SIGNAL_AVG]) {
			link.signal_average_dbm = static_cast<int8_t>(nla_get_u8(station[NL80211_STA_INFO_SIGNAL_AVG]));
		}
		link.tx_bitrate_mbps = station_bitrate(station[NL80211_STA_INFO_TX_BITRATE]);
		link.rx_bitrate_mbps = station_bitrate(station[NL80211_STA_INFO_RX_BITRATE]);
		if (station[NL80211_STA_INFO_EXPECTED_THROUGHPUT]) {
			link.expected_throughput_mbps = nla_get_u32(station[NL80211_STA_INFO_EXPECTED_THROUGHPUT]) / 1000.0;
		}
		if (station[NL80211_STA_INFO_INACTIVE_TIME]) {
			link.inactive_s = nla_get_u32(station[NL80211_STA_INFO_INACTIVE_TIME]) / 1000.0;
		}
		for (const auto& [field, counter] : std::vector<std::pair<int, uint64_t*>>{
				 {NL80211_STA_INFO_TX_PACKETS, &link.tx_packets}, {NL80211_STA_INFO_RX_PACKETS, &link.rx_packets},
				 {NL80211_STA_INFO_TX_RETRIES, &link.tx_retries}, {NL80211_STA_INFO_TX_FAILED, &link.tx_failed}}) {
			if (station[field]) {
				*counter = nla_get_u32(station[field]);
			}
		}
		if (station[NL80211_STA_INFO_RX_DROP_MISC]) {
			link.rx_dropped = nla_get_u64(station[NL80211_STA_INFO_RX_DROP_MISC]);
		}
		result.push_back(std::move(link));
	});
	return result;
}
}  // namespace mrs_uav_wifi_direct
