// SPDX-License-Identifier: BSD-3-Clause
/** @file wifi_discovery.hpp
 * @brief Find nearby group owners and check their channel settings.
 */
#pragma once

#include <optional>

#include "wifi_radio.hpp"
#include "wpa_control.hpp"

namespace mrs_uav_wifi_direct {
/** @brief Encode a validated public group SSID for the supplicant's literal join argument. */
inline std::string encode_ssid(const std::string& ssid) {
	constexpr char hex[] = "0123456789abcdef";
	std::string result;
	for (const unsigned char value : ssid) {
		result += hex[value >> 4];
		result += hex[value & 15];
	}
	return result;
}
/**
 * @brief Record a named UAV from an already fetched P2P/BSS record, independently of join eligibility.
 * Addresses identify observations, not trusted identities. Names, SSIDs and numeric
 * fields are bounded before logging; the status formatter expires cached records.
 * Reading advertisements from another fleet never adds it to the allowed UAV list.
 */
inline void observe_nearby(const std::string& record, const std::string& address, NearbyUavs& nearby) {
	std::smatch name, age, frequency, capability, ssid, device, signal, bssid;
	if (!std::regex_search(record, name,
			std::regex("(?:^|\\n)(?:p2p_device_name|wps_device_name|device_name)=(uav[0-9]{1,2})(?:\\n|$)")) ||
		std::stoi(name[1].str().substr(3)) == 0 ||
		!std::regex_search(record, age, std::regex("(?:^|\\n)age=([0-9]{1,4})(?:\\n|$)")) || std::stoi(age[1]) > 120) {
		return;
	}
	NearbyUav observation;
	observation.name = name[1];
	observation.device =
		std::regex_search(record, device, std::regex("(?:^|\\n)p2p_device_addr=([0-9A-Fa-f:]{17})(?:\\n|$)"))
		? device[1].str()
		: address;
	if (std::regex_search(record, device, std::regex("(?:^|\\n)interface_addr=([0-9A-Fa-f:]{17})(?:\\n|$)"))) {
		observation.interface_address = device[1];
	} else if (std::regex_search(record, bssid, std::regex("(?:^|\\n)bssid=([0-9A-Fa-f:]{17})(?:\\n|$)"))) {
		observation.interface_address = bssid[1];
	}
	for (auto* mac : {&observation.device, &observation.interface_address}) {
		std::transform(mac->begin(), mac->end(), mac->begin(), [](unsigned char value) { return std::tolower(value); });
	}
	observation.seen = Clock::now() - std::chrono::seconds(std::stoi(age[1]));
	if (std::regex_search(record, frequency, std::regex("(?:^|\\n)(?:oper_freq|freq)=([0-9]{4})(?:\\n|$)"))) {
		const auto value = std::stoul(frequency[1]);
		if (value >= 2400 && value <= 7125) {
			observation.frequency = value;
		}
	}
	RadioChannel advertised;
	bool pinned = false;
	if (!observation.frequency && peer_radio_model(record, advertised, pinned)) {
		observation.frequency = advertised.frequency;
	}
	observation.owner = std::regex_search(record, capability,
							std::regex("(?:^|\\n)(?:p2p_group_capab|group_capab)=0x([0-9a-fA-F]{1,2})(?:\\n|$)")) &&
		(std::stoul(capability[1], nullptr, 16) & 1);
	if (std::regex_search(record, ssid, std::regex("(?:^|\\n)ssid=(DIRECT-[A-Za-z0-9_.-]{2,25})(?:\\n|$)"))) {
		observation.ssid = ssid[1];
	}
	if (std::regex_search(record, signal, std::regex("(?:^|\\n)level=(-[0-9]{1,3})(?:\\n|$)")) &&
		std::stoi(signal[1]) >= -127) {
		observation.signal_dbm = std::stoi(signal[1]);
	}
	auto& current = nearby[observation.device];
	if (current.seen > observation.seen) {
		// A fresh beacon can add the SSID/RSSI omitted by a newer P2P record.
		if (current.owner && observation.owner && (!current.frequency || current.frequency == observation.frequency) &&
			Clock::now() - observation.seen <= 3s) {
			if (current.interface_address.empty()) {
				current.interface_address = observation.interface_address;
			}
			if (current.ssid.empty()) {
				current.ssid = observation.ssid;
			}
			if (!current.signal_dbm) {
				current.signal_dbm = observation.signal_dbm;
			}
			if (!current.frequency) {
				current.frequency = observation.frequency;
			}
		}
	} else {
		current = std::move(observation);
	}
}

/**
 * @brief Recover formed owners from fresh scan results when P2P_PEER is empty.
 * Some Full-MAC radios expose GO beacons only in the station BSS table. Require
 * both the P2P owner capability and configured P2P/WPS device name; an ordinary AP
 * with a similar SSID is never eligible. The group BSSID is a valid explicit
 * P2P_CONNECT join target even before the supplicant creates a peer record.
 */
inline DiscoveredOwners cached_owners(
	WpaControl& endpoint, const Config& config, const std::optional<RadioChannel>& pinned, NearbyUavs& nearby) {
	DiscoveredOwners result;
	const auto table = endpoint.request("SCAN_RESULTS");
	const std::regex entry("(?:^|\\n)([0-9A-Fa-f:]{17})\\t[^\\n]*\\[P2P\\][^\\n]*");
	for (auto row = std::sregex_iterator(table.begin(), table.end(), entry); row != std::sregex_iterator(); ++row) {
		const std::string bssid = (*row)[1];
		const auto bss = endpoint.request("BSS " + bssid);
		observe_nearby(bss, bssid, nearby);
		std::smatch age, capability, name, frequency, ssid;
		// A GO's beacon may omit its P2P Device Info name while carrying the
		// same configured name in WPS. Both are unauthenticated startup hints;
		// the P2P GO capability is still mandatory and TLS authenticates identity.
		const bool named = std::regex_search(bss, name, std::regex("(?:^|\\n)p2p_device_name=([^\\n]+)")) ||
			std::regex_search(bss, name, std::regex("(?:^|\\n)wps_device_name=([^\\n]+)"));
		if (!std::regex_search(bss, age, std::regex("(?:^|\\n)age=([0-9]{1,4})(?:\\n|$)")) || std::stoi(age[1]) > 3 ||
			!std::regex_search(bss, capability, std::regex("(?:^|\\n)p2p_group_capab=0x([0-9a-fA-F]{1,2})(?:\\n|$)")) ||
			!(std::stoul(capability[1], nullptr, 16) & 1) || !named ||
			!std::regex_search(bss, frequency, std::regex("(?:^|\\n)freq=([0-9]{4})(?:\\n|$)"))) {
			continue;
		}
		const auto value = static_cast<uint32_t>(std::stoul(frequency[1]));
		if (value < 2400 || value > 5885 || (pinned && value != pinned->frequency)) {
			continue;
		}
		std::string ssid_hex;
		if (std::regex_search(bss, ssid, std::regex("(?:^|\\n)ssid=(DIRECT-[A-Za-z0-9_.-]{2,25})(?:\\n|$)"))) {
			ssid_hex = encode_ssid(ssid[1]);
		}
		for (const auto& [number, full_name] : config.names) {
			if (number != config.own && full_name == name[1]) {
				result[number] = {
					bssid, Clock::now() - std::chrono::seconds(std::stoi(age[1])), {value, 0, 0, 0}, false, ssid_hex};
			}
		}
	}
	return result;
}

/**
 * @brief Recover a GO's actual frequency from a fresh, identity-matched cached BSS.
 * Full-MAC drivers sometimes omit oper_freq in P2P_PEER. A dedicated P2P device
 * and the managed station can have separate BSS caches, so inspect both endpoints.
 * Reading these caches never starts a scan or changes the AP association.
 */
inline uint32_t cached_group_frequency(
	const std::string& peer, const std::string& device, WpaControl& radio, WpaControl& station) {
	std::smatch interface;
	if (!std::regex_search(peer, interface, std::regex("(?:^|\\n)interface_addr=([0-9A-Fa-f:]{17})"))) {
		return 0;
	}
	for (auto* endpoint : {&radio, &station}) {
		const auto bss = endpoint->request("BSS " + interface[1].str());
		std::smatch age, frequency, address, capability;
		if (std::regex_search(bss, age, std::regex("(?:^|\\n)age=([0-9]{1,4})(?:\\n|$)")) && std::stoi(age[1]) <= 3 &&
			std::regex_search(bss, frequency, std::regex("(?:^|\\n)freq=([0-9]{4})(?:\\n|$)")) &&
			std::regex_search(bss, address, std::regex("(?:^|\\n)p2p_device_addr=([0-9A-Fa-f:]{17})")) &&
			address[1] == device &&
			std::regex_search(bss, capability, std::regex("(?:^|\\n)p2p_group_capab=0x([0-9a-fA-F]{1,2})(?:\\n|$)")) &&
			(std::stoul(capability[1], nullptr, 16) & 1)) {
			const auto value = std::stoul(frequency[1]);
			if (value >= 2400 && value <= 7125) {
				return value;
			}
		}
	}
	return 0;
}

/**
 * @brief Identify configured UAVs that recently advertised a formed P2P group.
 * Idle devices are not join targets. An existing AP constrains the complete
 * channel settings when advertised and at least the primary frequency otherwise. With
 * no AP, a client may follow a discovered owner's channel. Native attributes are
 * startup hints; mutual TLS subsequently authenticates the UAV and fleet state.
 */
inline DiscoveredOwners discover_owners(WpaControl& radio, WpaControl& station, const Config& config,
	const std::optional<RadioChannel>& pinned, NearbyUavs& nearby) {
	DiscoveredOwners result;
	auto peer = radio.request("P2P_PEER FIRST");
	for (int index = 0; index < 128 && peer.compare(0, 4, "FAIL") != 0; ++index) {
		std::smatch address, age, capability, name, operating;
		if (!std::regex_search(peer, address, std::regex("^([0-9A-Fa-f:]{17})"))) {
			break;
		}
		const std::string device = address[1];
		observe_nearby(peer, device, nearby);
		if (std::regex_search(peer, age, std::regex("(?:^|\\n)age=([0-9]{1,4})(?:\\n|$)")) && std::stoi(age[1]) <= 3 &&
			std::regex_search(peer, capability, std::regex("(?:^|\\n)group_capab=0x([0-9a-fA-F]{1,2})(?:\\n|$)")) &&
			(std::stoul(capability[1], nullptr, 16) & 1) &&
			std::regex_search(peer, name, std::regex("(?:^|\\n)device_name=([^\\n]+)"))) {
			uint32_t frequency = 0;
			if (std::regex_search(peer, operating, std::regex("(?:^|\\n)oper_freq=([0-9]{4})(?:\\n|$)"))) {
				frequency = std::stoul(operating[1]);
			}
			if (frequency < 2400 || frequency > 7125) {
				frequency = cached_group_frequency(peer, device, radio, station);
			}
			RadioChannel advertised;
			bool ap_present = false;
			bool advertised_pinned = false;
			const bool metadata = peer_radio_model(peer, advertised, advertised_pinned);
			// Full-MAC radios can discover a formed GO without exposing its BSS
			// through either control endpoint. Its fresh P2P advertisement carries
			// our validated channel settings too. Use that only when no measured
			// frequency exists; never override a conflicting actual channel.
			// GROUP-STARTED subsequently verifies the selected AP-pinned channel,
			// and mutual TLS authenticates the peer before accepting any data.
			if (!frequency && metadata) {
				frequency = advertised.frequency;
			}
			RadioChannel observed{frequency, 0, 0, 0};
			if (metadata && advertised.frequency == frequency) {
				observed = advertised;
				ap_present = advertised_pinned;
			}
			const bool compatible = !pinned || (observed.width ? observed == *pinned : frequency == pinned->frequency);
			if (frequency && compatible) {
				for (const auto& [number, full_name] : config.names) {
					if (number != config.own && full_name == name[1]) {
						result[number] = {
							device, Clock::now() - std::chrono::seconds(std::stoi(age[1])), observed, ap_present};
					}
				}
			}
		}
		peer = radio.request("P2P_PEER NEXT-" + device);
	}
	for (auto* endpoint : {&radio, &station}) {
		for (const auto& [number, observation] : cached_owners(*endpoint, config, pinned, nearby)) {
			// Retain native channel settings, but include the actual fresh beacon's
			// SSID/BSSID. An explicit group SSID avoids wildcard join scans,
			// which some Full-MAC drivers cannot execute on the operating
			// channel. Supplicant still handles WPS provisioning. Never merge channels.
			auto [entry, inserted] = result.emplace(number, observation);
			if (!inserted && !observation.ssid.empty() &&
				entry->second.radio.frequency == observation.radio.frequency) {
				entry->second.ssid = observation.ssid;
				entry->second.device = observation.device;
			}
		}
	}
	return result;
}
}  // namespace mrs_uav_wifi_direct
