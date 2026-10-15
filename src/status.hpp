// SPDX-License-Identifier: BSD-3-Clause
/** @file status.hpp
 * @brief Snapshots and compact formatting for the node's periodic status log.
 */
#pragma once

#include <iomanip>
#include <sstream>

#include "config.hpp"
#include "fleet_policy.hpp"
#include "network_utils.hpp"

namespace mrs_uav_wifi_direct {
/** @brief Cumulative per-peer/topic samples accepted or dropped locally, across reconnects. */
struct TopicTraffic {
	uint64_t messages{}, drops{};
};
/** @brief Passive QUIC round-trip estimates in microseconds, sampled on its worker. */
struct PathLatency {
	uint32_t rtt{}, minimum{}, maximum{}, variation{};
	Clock::time_point updated{};
};
/** @brief Authenticated connections currently available to a single UAV. */
struct PeerConnections {
	bool ap{}, direct{};
	PathLatency ap_latency, direct_latency;
};
/** @brief Copy transport state while holding its callback mutex; format it after unlocking. */
struct TransportStatus {
	InterfaceNetwork ap, direct;
	bool ap_listening{}, direct_listening{};
	std::map<int, PeerConnections> connected;
	FleetPeers peers;
	std::map<std::pair<int, std::string>, TopicTraffic> exported;
	std::map<std::pair<int, std::string>, uint64_t> receive_drops_by_topic;
	size_t send_bytes{}, receive_bytes{}, receive_messages{}, streams{}, pending_connections{};
	uint64_t send_drops{}, receive_drops{};
};
/** @brief Copy the radio worker's current group, recent discoveries, and setup error. */
struct WifiStatus {
	FleetStatus fleet;
	DiscoveredOwners discovered;
	NearbyUavs nearby;
	std::string group, error;
};

/** @brief Keep small rates visible with three significant digits. */
inline std::string status_number(double value) {
	std::ostringstream out;
	out << std::setprecision(3) << value;
	return out.str();
}
/** @brief Describe how long ago a sample/report arrived, using the steady clock. */
inline std::string status_age(Clock::time_point last, Clock::time_point now) {
	return last == Clock::time_point{} ? "never"
									   : status_number(std::chrono::duration<double>(now - last).count()) + "s";
}
/** @brief Format smoothed RTT, lifetime minimum/maximum and RTT variation, in milliseconds. */
inline std::string status_latency(const PathLatency& latency, Clock::time_point now, int timeout) {
	if (latency.updated == Clock::time_point{}) {
		return "unknown";
	}
	if (now - latency.updated > std::chrono::seconds(timeout)) {
		return "stale";
	}
	return status_number(latency.rtt / 1000.0) + "ms(min=" + status_number(latency.minimum / 1000.0) +
		",max=" + status_number(latency.maximum / 1000.0) + ",var=" + status_number(latency.variation / 1000.0) + ")";
}
/** @brief Join a bounded fleet/topic list, retaining an explicit empty value. */
inline std::string status_list(const std::vector<std::string>& entries) {
	std::string result = "[";
	for (const auto& entry : entries) {
		if (result.size() > 1) {
			result += ", ";
		}
		result += entry;
	}
	return result + "]";
}
/** @brief Match the same direct-first delivery policy used by QuicTransport::send(). */
inline const char* status_route(const Config& config, const PeerConnections& peer) {
	return peer.direct ? "direct" : peer.ap && config.ap && config.fallback ? "AP" : "no_route";
}
/** @brief Name the worker's group role in ordinary log output. */
inline const char* status_role(FleetRole role) {
	switch (role) {
		case FleetRole::hosting:
			return "owner";
		case FleetRole::joining:
			return "joining";
		case FleetRole::client:
			return "client";
		default:
			return "idle";
	}
}
/** @brief Distinguish a disabled transport, an unavailable network and a pending listener. */
inline std::string status_network(bool enabled, const InterfaceNetwork& network, bool listening) {
	if (!enabled) {
		return "disabled";
	}
	return network ? network.interface + "@" + network.address + (listening ? "(ready)" : "(listener_pending)")
				   : "unavailable";
}
/**
 * @brief Format configuration, paths, discovery and health without querying the radio.
 * Authenticated QUIC paths determine connection health. Wi-Fi advertisements are
 * public discovery hints and expire after peer_timeout_s; up to eight nearby UAVs
 * are listed with a count for additional observations. The ROS executor appends
 * one traffic row per configured peer using its topic counters and publishers.
 */
inline std::string status_summary(const Config& config, const std::string& path, const WifiStatus& wifi,
	const TransportStatus& transport, Clock::time_point now, uint64_t new_drops, uint64_t publish_errors,
	bool missing_source) {
	std::ostringstream out;
	std::vector<std::string> configured, detected, nearby;
	size_t routes = 0, direct_peers = 0, reporting = 0;
	for (const auto peer : config.numbers) {
		configured.push_back(config.name(peer));
		if (peer == config.own) {
			continue;
		}
		const auto link = transport.connected.find(peer);
		const auto report = transport.peers.find(peer);
		const auto hint = wifi.discovered.find(peer);
		std::vector<std::string> observations;
		if (report != transport.peers.end()) {
			observations.push_back("TLS=" + status_age(report->second.seen, now));
		} else if (link != transport.connected.end()) {
			observations.push_back("TLS");
		}
		if (hint != wifi.discovered.end() && now - hint->second.seen <= std::chrono::seconds(config.peer_timeout)) {
			observations.push_back("WiFi=" + status_age(hint->second.seen, now));
		}
		if (!observations.empty()) {
			detected.push_back(config.name(peer) + status_list(observations));
		}
		if (link != transport.connected.end()) {
			const auto& paths = link->second;
			routes += paths.direct || (paths.ap && config.ap && config.fallback);
			direct_peers += paths.direct;
			reporting += report != transport.peers.end();
		}
	}
	size_t observed = 0;
	for (const auto& [device, advertisement] : wifi.nearby) {
		if (now - advertisement.seen > std::chrono::seconds(config.peer_timeout)) {
			continue;
		}
		if (++observed > 8) {
			continue;
		}
		std::string entry = advertisement.name + "@" + device + (advertisement.owner ? "(owner" : "(device");
		if (!advertisement.ssid.empty()) {
			entry += ",group=" + advertisement.ssid;
		}
		if (advertisement.frequency) {
			entry += "," + std::to_string(advertisement.frequency) + "MHz";
		}
		if (advertisement.signal_dbm) {
			entry += "," + std::to_string(*advertisement.signal_dbm) + "dBm";
		}
		nearby.push_back(entry + ",age=" + status_age(advertisement.seen, now) + ")");
	}
	if (observed > nearby.size()) {
		nearby.push_back("+" + std::to_string(observed - nearby.size()) + " more");
	}
	if (config.numbers.empty()) {
		out << "Status DISCOVERY_ONLY " << config.hostname << " | config=" << path
			<< " | uav_list=[] interface=" << config.interface << " nearby(unverified/cache)=" << status_list(nearby)
			<< " discovery=/" << config.hostname << "/wifi_direct/discovery";
		return out.str();
	}
	const auto peers = config.numbers.size() - 1;
	const bool direct_ready = transport.direct && transport.direct_listening;
	const char* health = !peers ? "IDLE"
		: !routes				? "WAITING_PEERS"
		: (routes < peers || reporting < peers || (config.wifi && (!direct_ready || direct_peers < peers)) ||
			  !wifi.error.empty() || new_drops || publish_errors || missing_source)
		? "DEGRADED"
		: "OK";
	out << "Status " << health << " " << config.name(config.own) << " | config=" << path
		<< " | fleet=" << status_list(configured) << " port=" << config.listen_port(config.own)
		<< " TLS=" << (config.ca.empty() ? "peer_pinning" : "CA") << " max_message=" << config.max_message
		<< "B stream/receive_limit=" << config.max_pending << "B"
		<< " interface=" << config.interface << " peer_timeout=" << config.peer_timeout << "s";
	out << "\n  links: AP=" << status_network(config.ap, transport.ap, transport.ap_listening)
		<< " fallback=" << (config.fallback ? "on" : "off")
		<< " | direct=" << status_network(config.wifi, transport.direct, transport.direct_listening);
	if (config.wifi) {
		out << " mode=" << (config.existing_interface.empty() ? "managed" : "external")
			<< " group=" << (wifi.group.empty() ? "none" : wifi.group) << " DSCP=" << config.direct_dscp;
		if (config.existing_interface.empty()) {
			out << " role=" << status_role(wifi.fleet.role)
				<< " owner=" << (wifi.fleet.owner ? config.name(wifi.fleet.owner) : "none")
				<< " ready=" << (wifi.fleet.ready ? "yes" : "no") << " channel=" << wifi.fleet.radio.frequency << "MHz/"
				<< wifi.fleet.radio.width << "MHz"
				<< " AP_pinned=" << (wifi.fleet.ap_present ? "yes" : "no")
				<< " can_host=" << (wifi.fleet.can_host ? "yes" : "no")
				<< " can_join=" << (wifi.fleet.can_join ? "yes" : "no") << " members=" << wifi.fleet.members;
		}
	}
	out << "\n  detected=" << status_list(detected) << " nearby(unverified/cache)=" << status_list(nearby)
		<< " pending=" << transport.pending_connections;
	out << "\n  queues: send=" << transport.send_bytes << "B receive=" << transport.receive_bytes << "B/"
		<< transport.receive_messages << "msg streams=" << transport.streams
		<< " | local_drops(total): send=" << transport.send_drops << " receive=" << transport.receive_drops
		<< " recent=" << new_drops << " publish_errors_recent=" << publish_errors;
	if (!wifi.error.empty()) {
		auto error = wifi.error;
		std::replace(error.begin(), error.end(), '\n', ' ');
		out << "\n  direct_error: " << error;
	}
	return out.str();
}
}  // namespace mrs_uav_wifi_direct
