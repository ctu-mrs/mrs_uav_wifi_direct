// SPDX-License-Identifier: BSD-3-Clause
/** @file discovery_message.hpp
 * @brief Combine unverified radio observations and authenticated fleet state for ROS publication.
 */
#pragma once

#include <mrs_uav_wifi_direct/msg/discovery.hpp>

#include "radio_discovery.hpp"
#include "status.hpp"

namespace mrs_uav_wifi_direct {
/** @brief Convert a monotonic observation age to seconds; NaN means no sample has arrived. */
inline double observation_age(Clock::time_point sampled, Clock::time_point now) {
	return sampled == Clock::time_point{} ? std::numeric_limits<double>::quiet_NaN()
										  : std::max(0.0, std::chrono::duration<double>(now - sampled).count());
}
/** @brief Merge public advertisements by MAC and retain the most recently observed record. */
inline NearbyUavs nearby_observations(
	const RadioSnapshot& radio, const WifiStatus& wifi, Clock::time_point now, int timeout) {
	auto observations = radio.nearby;
	for (const auto& [address, entry] : wifi.nearby) {
		const auto current = observations.find(address);
		if (current == observations.end() || current->second.seen < entry.seen) {
			observations[address] = entry;
		}
	}
	for (auto item = observations.begin(); item != observations.end();) {
		if (now - item->second.seen > std::chrono::seconds(timeout)) {
			item = observations.erase(item);
		} else {
			++item;
		}
	}
	return observations;
}
/** @brief Fill an authenticated path's latency; old measurements keep their age and are marked unknown. */
inline void path_latency(
	msg::LinkMetrics& link, bool connected, const PathLatency& latency, Clock::time_point now, int timeout) {
	link.connected = connected;
	link.rtt_age_s = observation_age(latency.updated, now);
	if (connected && latency.updated != Clock::time_point{} && now - latency.updated <= std::chrono::seconds(timeout)) {
		link.rtt_ms = latency.rtt / 1000.0;
		link.minimum_rtt_ms = latency.minimum / 1000.0;
		link.maximum_rtt_ms = latency.maximum / 1000.0;
		link.rtt_variation_ms = latency.variation / 1000.0;
	}
}
/**
 * @brief Assemble a snapshot without radio queries or nested worker locks.
 * Configured peers remain visible while disconnected. Other UAVs appear only while
 * their cached advertisement is fresh. Multiple devices advertising the same name
 * remain separate observations. Authentication flags describe QUIC identity; they
 * do not authenticate the public name, RSSI or MAC in an advertisement.
 */
inline msg::Discovery discovery_message(const Config& config, const RadioSnapshot& radio, const WifiStatus& wifi,
	const TransportStatus& transport, Clock::time_point now) {
	msg::Discovery output;
	output.hostname = config.hostname;
	output.interface = config.interface;
	output.driver = radio.driver;
	output.phy = radio.phy;
	output.communication_enabled = !config.numbers.empty();
	output.radio_available = radio.available;
	output.radio_age_s = observation_age(radio.sampled, now);
	output.radio_error = radio.error;
	output.discovery_error = radio.discovery_error;
	output.direct_error = wifi.error;
	output.group_interface = wifi.group;
	output.group_role = status_role(wifi.fleet.role);
	output.group_owner = wifi.fleet.owner ? config.name(wifi.fleet.owner) : "";
	output.group_ready = wifi.fleet.ready;
	output.ap = radio.ap;
	output.direct_stations = radio.stations;
	const auto nan = std::numeric_limits<double>::quiet_NaN();
	for (const auto& [address, observation] : nearby_observations(radio, wifi, now, config.peer_timeout)) {
		msg::DiscoveredUav peer;
		peer.hostname = observation.name;
		peer.advertisement_present = true;
		peer.device_address = address;
		peer.interface_address = observation.interface_address;
		peer.group_ssid = observation.ssid;
		peer.group_owner = observation.owner;
		peer.frequency_mhz = observation.frequency;
		peer.signal_dbm = observation.signal_dbm ? *observation.signal_dbm : nan;
		peer.advertisement_age_s = observation_age(observation.seen, now);
		output.uavs.push_back(std::move(peer));
	}
	for (const auto number : config.numbers) {
		if (number != config.own && std::none_of(output.uavs.begin(), output.uavs.end(), [&](const auto& peer) {
				return peer.hostname == config.name(number);
			})) {
			msg::DiscoveredUav peer;
			peer.hostname = config.name(number);
			peer.signal_dbm = peer.advertisement_age_s = nan;
			output.uavs.push_back(std::move(peer));
		}
	}
	for (auto& peer : output.uavs) {
		peer.ap = empty_link();
		peer.direct = empty_link();
		peer.heartbeat_age_s = nan;
		// Station tables use the group interface address, which may differ from
		// the public P2P device address. Match exact addresses; RSSI from the
		// infrastructure AP is never substituted for a UAV's direct-link RSSI.
		for (const auto& station : radio.stations) {
			if (station.mac_address == peer.interface_address || station.mac_address == peer.device_address) {
				peer.direct = station;
				break;
			}
		}
		for (const auto& [number, name] : config.names) {
			if (number == config.own || name != peer.hostname) {
				continue;
			}
			peer.configured = true;
			const auto connection = transport.connected.find(number);
			if (connection != transport.connected.end()) {
				peer.authenticated_ap = connection->second.ap;
				peer.authenticated_direct = connection->second.direct;
				path_latency(peer.ap, peer.authenticated_ap, connection->second.ap_latency, now, config.peer_timeout);
				path_latency(peer.direct, peer.authenticated_direct, connection->second.direct_latency, now,
					config.peer_timeout);
			}
			for (const auto& [network, link] : std::vector<std::pair<const InterfaceNetwork*, msg::LinkMetrics*>>{
					 {&transport.ap, &peer.ap}, {&transport.direct, &peer.direct}}) {
				if (*network) {
					link->interface = network->interface;
					link->address = network->peer_address(config.own, number);
				}
			}
			const auto report = transport.peers.find(number);
			if (report != transport.peers.end() &&
				now - report->second.seen <= std::chrono::seconds(config.peer_timeout)) {
				const auto& fleet = report->second.status;
				peer.heartbeat_age_s = observation_age(report->second.seen, now);
				peer.fleet_role = status_role(fleet.role);
				peer.fleet_owner = fleet.owner ? config.name(fleet.owner) : "";
				peer.fleet_members = fleet.members;
				peer.fleet_ready = fleet.ready;
				peer.fleet_can_host = fleet.can_host;
				peer.fleet_can_join = fleet.can_join;
			}
		}
	}
	return output;
}
}  // namespace mrs_uav_wifi_direct
