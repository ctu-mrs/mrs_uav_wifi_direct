// SPDX-License-Identifier: BSD-3-Clause
/** @file fleet_policy.hpp
 * @brief Choose a group owner and encode fleet status messages.
 */
#pragma once

#include <algorithm>
#include <map>
#include <set>
#include <tuple>

#include "common.hpp"

namespace mrs_uav_wifi_direct {
/** @brief Channel frequency, width, and centers shared by the AP and P2P group. */
struct RadioChannel {
	uint32_t frequency{}, center1{}, center2{};
	uint16_t width{};
	/** @brief Require an identical primary frequency, width and both centers. */
	bool operator==(const RadioChannel& other) const {
		return std::tie(frequency, width, center1, center2) ==
			std::tie(other.frequency, other.width, other.center1, other.center2);
	}
};
/** @brief Group state reported to peers; joining completes after an address is assigned. */
enum class FleetRole : uint8_t { idle, hosting, joining, client };
/** @brief A UAV reports only its own capabilities, current group and AP channel. */
struct FleetStatus {
	RadioChannel radio;
	int owner{};
	// A host reports its authenticated member count so another group need not
	// reach every client individually to compare group sizes consistently.
	uint16_t members{};
	FleetRole role{FleetRole::idle};
	bool can_host{}, can_join{}, ready{}, ap_present{};
};
/** @brief Measure report age with this host's steady clock. */
struct FleetObservation {
	FleetStatus status;
	uint64_t instance{}, sequence{};
	Clock::time_point seen{};
};
/** @brief A recent Wi-Fi advertisement helps find the group; TLS verifies the UAV. */
struct DiscoveredOwner {
	std::string device;
	Clock::time_point seen{};
	RadioChannel radio;
	bool ap_present{};
	// Optional exact SSID, encoded as hex for the supplicant's literal join argv.
	std::string ssid{};
};
using FleetPeers = std::map<int, FleetObservation>;
using DiscoveredOwners = std::map<int, DiscoveredOwner>;

/** @brief A connected AP pins the full channel; standalone clients may follow any usable owner. */
inline bool fleet_channel_compatible(const FleetStatus& local, const FleetStatus& peer) {
	return !local.ap_present || local.radio == peer.radio ||
		(!peer.ap_present && peer.radio.frequency == local.radio.frequency && peer.role == FleetRole::client);
}

/**
 * @brief Choose an available, compatible owner without a configured leader.
 * Ready groups outrank idle candidates. Prefer the group with the most observed
 * members, breaking ties by UAV number; empty standalone groups first prefer
 * 5 GHz when available. A returning idle UAV cannot replace an existing group.
 * With no group, the lowest eligible, recently heard UAV hosts. Expired reports,
 * incompatible AP channels and hosts waiting to retry are excluded. Callers hold a
 * changed decision for switch_hold_s before acting, avoiding transient churn.
 * This policy operates on reachable peers: disconnected partitions may each form
 * a group and converge when authenticated connectivity returns.
 */
inline int elect_owner(int own, const FleetStatus& local, const FleetPeers& peers, const DiscoveredOwners& discovered,
	Clock::time_point now, std::chrono::seconds timeout) {
	std::map<int, FleetStatus> live{{own, local}};
	for (const auto& [peer, observation] : peers) {
		if (now - observation.seen <= timeout && fleet_channel_compatible(local, observation.status)) {
			live[peer] = observation.status;
		}
	}
	std::map<int, int> groups;
	for (const auto& [peer, status] : live) {
		if (status.role == FleetRole::hosting && status.ready && status.can_host && status.owner == peer) {
			groups[peer] = std::max(1, static_cast<int>(status.members));
		}
	}
	for (const auto& [peer, hint] : discovered) {
		if (now - hint.seen <= timeout && peer != own &&
			(!local.ap_present || !hint.radio.frequency ||
				(hint.radio.width ? hint.radio == local.radio : hint.radio.frequency == local.radio.frequency))) {
			// A fresh beacon proves that a group exists even before any IP path does.
			// Ignore a newer authenticated report saying that this peer stopped hosting.
			const auto report = live.find(peer);
			if (report == live.end() || report->second.role == FleetRole::hosting) {
				groups.emplace(peer, 1);
			}
		}
	}
	std::map<int, int> observed;
	for (const auto& [peer, count] : groups) {
		observed[peer] = 1;
	}
	for (const auto& [peer, status] : live) {
		if (status.role == FleetRole::client && status.ready && groups.count(status.owner)) {
			++observed[status.owner];
		}
	}
	for (const auto& [peer, count] : observed) {
		groups[peer] = std::max(groups[peer], count);
	}
	int selected = 0, members = 0;
	bool selected_pinned = false, selected_fast = false;
	for (const auto& [peer, count] : groups) {
		const auto report = live.find(peer);
		const auto hint = discovered.find(peer);
		bool pinned =
			report != live.end() ? report->second.ap_present : hint != discovered.end() && hint->second.ap_present;
		const bool fast = (report != live.end()				 ? report->second.radio.frequency
								  : hint != discovered.end() ? hint->second.radio.frequency
															 : 0) >= 5000;
		for (const auto& [member, status] : live) {
			pinned |= status.role == FleetRole::client && status.ready && status.owner == peer && status.ap_present;
		}
		// Flexible radios follow AP-constrained groups, whose channel cannot move.
		// Among equally constrained groups retain the size/identity policy.
		// An empty standalone 5 GHz group is a better startup target than
		// an empty 2.4 GHz group. Preserve groups that already have clients;
		// AP channel constraints and member counts retain their precedence.
		if (!selected || (pinned && !selected_pinned) ||
			(pinned == selected_pinned &&
				(count > members || (!pinned && count == 1 && members == 1 && fast && !selected_fast)))) {
			selected = peer;
			members = count;
			selected_pinned = pinned;
			selected_fast = fast;
		}
	}
	if (selected) {
		return selected;
	}
	for (const auto& [peer, status] : live) {
		if (status.can_host) {
			return peer;
		}
	}
	return 0;
}

/** @brief Hash the fleet settings to detect configuration mismatches. */
inline uint64_t fleet_hash(const std::vector<std::string>& fields) {
	uint64_t result = 14695981039346656037ULL;
	for (const auto& field : fields) {
		for (const auto byte : field) {
			result = (result ^ static_cast<uint8_t>(byte)) * 1099511628211ULL;
		}
		result = (result ^ 0xff) * 1099511628211ULL;
	}
	return result;
}
/** @brief Fixed 36-byte report; independent of topic framing, queues and rate limits. */
inline std::vector<uint8_t> fleet_record(const FleetStatus& status, uint64_t sequence, uint64_t hash) {
	std::vector<uint8_t> bytes;
	append(bytes, sequence, 8);
	append(bytes, hash, 8);
	append(bytes, status.radio.frequency, 4);
	append(bytes, status.radio.width, 2);
	append(bytes, status.radio.center1, 4);
	append(bytes, status.radio.center2, 4);
	append(bytes, status.owner, 2);
	append(bytes, static_cast<uint8_t>(status.role), 1);
	append(bytes, status.can_host | (status.can_join << 1) | (status.ready << 2) | (status.ap_present << 3), 1);
	append(bytes, status.members, 2);
	return bytes;
}
/**
 * @brief Decode a validated-size record and reject inconsistent identities/state.
 * Certificate identity is checked by the transport before this helper runs. The hash checks that both UAVs use the same
 * fleet settings; peer trust comes from the certificate check.
 */
inline bool decode_fleet(
	const uint8_t* bytes, int peer, const std::set<int>& allowed, uint64_t hash, FleetStatus& status) {
	if (integer(bytes + 8, 8) != hash || bytes[32] > static_cast<uint8_t>(FleetRole::client) || bytes[33] > 15) {
		return false;
	}
	status.radio = {static_cast<uint32_t>(integer(bytes + 16, 4)), static_cast<uint32_t>(integer(bytes + 22, 4)),
		static_cast<uint32_t>(integer(bytes + 26, 4)), static_cast<uint16_t>(integer(bytes + 20, 2))};
	status.owner = integer(bytes + 30, 2);
	status.role = static_cast<FleetRole>(bytes[32]);
	status.can_host = bytes[33] & 1;
	status.can_join = bytes[33] & 2;
	status.ready = bytes[33] & 4;
	status.ap_present = bytes[33] & 8;
	status.members = integer(bytes + 34, 2);
	if ((status.owner && !allowed.count(status.owner)) || status.members > allowed.size() ||
		(status.role != FleetRole::hosting && status.members != 0)) {
		return false;
	}
	const bool active = status.role != FleetRole::idle;
	const bool host = status.role == FleetRole::hosting;
	const bool channel = status.radio.frequency >= 2400 && status.radio.frequency <= 7125 &&
		(status.radio.width == 20 || status.radio.width == 40 || status.radio.width == 80 ||
			status.radio.width == 160) &&
		status.radio.center1 >= 2400 && status.radio.center1 <= 7125 &&
		(status.radio.center2 == 0 || (status.radio.center2 >= 2400 && status.radio.center2 <= 7125));
	return (!active || status.owner != 0) && (!host || (status.owner == peer && status.can_host)) &&
		(!(status.role == FleetRole::client || status.role == FleetRole::joining) ||
			(status.owner != peer && status.can_join)) &&
		(!status.ready || status.role == FleetRole::hosting || status.role == FleetRole::client) &&
		(!(status.can_host || status.can_join || active) || channel);
}
}  // namespace mrs_uav_wifi_direct
