// SPDX-License-Identifier: BSD-3-Clause
/** @file wifi_radio.hpp
 * @brief Check radio concurrency and permitted channels before creating a group.
 */
#pragma once

#include <sys/utsname.h>

#include <cctype>
#include <filesystem>
#include <fstream>
#include <regex>
#include <stdexcept>

#include "fleet_policy.hpp"

namespace mrs_uav_wifi_direct {
/** @brief Encode bounded native-discovery metadata; TLS authenticates subsequent fleet reports. */
inline std::string radio_model(const RadioChannel& radio, bool pinned) {
	return "mrs-wfd:" + std::to_string(pinned) + ":" + std::to_string(radio.frequency) + ":" +
		std::to_string(radio.width) + ":" + std::to_string(radio.center1) + ":" + std::to_string(radio.center2);
}

/** @brief Read optional WPS model metadata without accepting unbounded numbers or unsupported channel settings. */
inline bool peer_radio_model(const std::string& peer, RadioChannel& radio, bool& pinned) {
	std::smatch match;
	const std::regex model(
		"(?:^|\\n)model_name=mrs-wfd:([01]):([0-9]{4}):(20|40|80|160):([0-9]{4}):([0-9]{1,4})(?:\\n|$)");
	if (!std::regex_search(peer, match, model)) {
		return false;
	}
	RadioChannel value{static_cast<uint32_t>(std::stoul(match[2])), static_cast<uint32_t>(std::stoul(match[4])),
		static_cast<uint32_t>(std::stoul(match[5])), static_cast<uint16_t>(std::stoul(match[3]))};
	if (value.frequency < 2400 || value.frequency > 7125 || value.center1 < 2400 || value.center1 > 7125 ||
		value.center2) {
		return false;
	}
	const int first = static_cast<int>(value.center1) - value.width / 2 + 10;
	if (static_cast<int>(value.frequency) < first || static_cast<int>(value.frequency) >= first + value.width ||
		(static_cast<int>(value.frequency) - first) % 20) {
		return false;
	}
	radio = value;
	pinned = match[1] == "1";
	return true;
}

/**
 * @brief Prefer a legal non-radar 5 GHz standalone channel, falling back to social channels.
 * Wider-band-capable firmware need not be forced onto a congested 2.4 GHz band.
 * NO_IR and DFS remain prohibited without an AP, even if concurrent hosting is supported.
 */
inline RadioChannel standalone_channel(const std::string& channels) {
	for (const int frequency : {5180, 5200, 5220, 5240, 2412, 2437, 2462}) {
		const RadioChannel radio{static_cast<uint32_t>(frequency), static_cast<uint32_t>(frequency), 0, 20};
		const std::regex pattern(
			"(?:^|\\n)[ \\t]*\\* " + std::to_string(frequency) + " MHz[^\\n]*(?:\\n[ \\t]+[A-Za-z][^\\n]*)*");
		std::smatch match;
		if (!std::regex_search(channels, match, pattern)) {
			continue;
		}
		auto text = match.str();
		std::transform(text.begin(), text.end(), text.begin(), [](unsigned char value) { return std::tolower(value); });
		if (text.find("disabled") == std::string::npos && text.find("no ir") == std::string::npos &&
			text.find("no-ir") == std::string::npos && text.find("radar") == std::string::npos) {
			return radio;
		}
	}
	return {};
}

/**
 * @brief Build bounded, supported-channel discovery targets without a full-band scan.
 * Social channels cover standalone fleet startup. A remembered infrastructure
 * channel goes first to recover peers that still have their AP; remaining 5 GHz
 * channels allow a newly booted standalone client to find such a fleet too.
 * Skip disabled/radar channels and 6 GHz, which this P2P implementation does not
 * negotiate. NO_IR channels remain valid client discovery/join targets.
 */
inline std::vector<uint32_t> discovery_channels(const std::string& channels, uint32_t preferred) {
	std::set<uint32_t> available;
	const std::regex pattern("(?:^|\\n)[ \\t]*\\* ([0-9]{4}) MHz[^\\n]*(?:\\n[ \\t]+[A-Za-z][^\\n]*)*");
	for (auto item = std::sregex_iterator(channels.begin(), channels.end(), pattern); item != std::sregex_iterator();
		++item) {
		const auto frequency = static_cast<uint32_t>(std::stoul((*item)[1]));
		auto text = item->str();
		std::transform(text.begin(), text.end(), text.begin(), [](unsigned char value) { return std::tolower(value); });
		if ((frequency == 2412 || frequency == 2437 || frequency == 2462 || (frequency >= 5180 && frequency <= 5885)) &&
			text.find("disabled") == std::string::npos && text.find("radar") == std::string::npos) {
			available.insert(frequency);
		}
	}
	std::vector<uint32_t> result;
	for (const auto frequency : {preferred, 2412U, 2437U, 2462U}) {
		if (available.erase(frequency)) {
			result.push_back(frequency);
		}
	}
	result.insert(result.end(), available.begin(), available.end());
	if (result.empty()) {
		throw std::runtime_error("No supported non-radar P2P discovery channel");
	}
	return result;
}

/**
 * @brief Check iw interface combinations for one managed and one P2P role.
 * Both roles must fit each advertised group limit and the total interface limit.
 * Unknown/missing capabilities reject unsupported input; hardware restrictions remain enforced
 * by the driver during group creation. Only same-channel concurrency is requested.
 */
inline bool concurrent_combination(const std::string& info, bool owner) {
	const auto start = info.find("valid interface combinations:");
	if (start == std::string::npos) {
		return false;
	}
	const auto text = info.substr(start);
	const std::regex group("#\\{([^}]+)\\}\\s*<=\\s*([0-9]+)");
	const std::regex total("total\\s*<=\\s*([0-9]+)");
	const auto target = owner ? "P2P-GO" : "P2P-client";
	size_t position = text.find('*');
	while (position != std::string::npos) {
		const auto end = text.find('*', position + 1);
		const auto combo = text.substr(position, end == std::string::npos ? end : end - position);
		bool managed = false, p2p = false, allowed = true;
		std::smatch count;
		if (std::regex_search(combo, count, total) && std::stoi(count[1]) >= 2) {
			for (auto it = std::sregex_iterator(combo.begin(), combo.end(), group); it != std::sregex_iterator();
				++it) {
				const std::string types = (*it)[1];
				const bool station = types.find("managed") != std::string::npos;
				const bool direct = types.find(target) != std::string::npos;
				managed |= station;
				p2p |= direct;
				if (static_cast<int>(station) + static_cast<int>(direct) > std::stoi((*it)[2])) {
					allowed = false;
				}
			}
			if (managed && p2p && allowed) {
				return true;
			}
		}
		position = end;
	}
	return false;
}

/**
 * @brief Convert the AP's primary 20 MHz channel to its global operating class.
 * P2P discovery and listen operations use this channel as well as the data group.
 * Unknown bands fail rather than falling back to off-channel social discovery.
 */
inline std::pair<int, int> ap_listen_channel(int frequency) {
	if (frequency >= 2412 && frequency <= 2472 && (frequency - 2407) % 5 == 0) {
		return {(frequency - 2407) / 5, 81};
	}
	if (frequency == 2484) {
		return {14, 82};
	}
	if (frequency >= 5180 && frequency <= 5240 && (frequency - 5180) % 20 == 0) {
		return {(frequency - 5000) / 5, 115};
	}
	if (frequency >= 5260 && frequency <= 5320 && (frequency - 5260) % 20 == 0) {
		return {(frequency - 5000) / 5, 118};
	}
	if (frequency >= 5500 && frequency <= 5720 && (frequency - 5500) % 20 == 0) {
		return {(frequency - 5000) / 5, 121};
	}
	if (frequency >= 5745 && frequency <= 5885 && (frequency - 5745) % 20 == 0) {
		return {(frequency - 5000) / 5, 125};
	}
	throw std::runtime_error("AP channel cannot be used for constrained P2P discovery");
}

/**
 * @brief Compare the complete channel definitions reported by iw, rejecting unsupported input.
 * Matching the primary frequency alone is insufficient: different widths or
 * center frequencies can make firmware switch between channel contexts. Compare
 * the optional second center as well, and ignore unrelated interface properties.
 */
inline bool same_radio_channel(const std::string& first, const std::string& second) {
	const std::regex channel(
		"channel [0-9]+ \\([0-9]+ MHz\\), width: [0-9]+ MHz, center1: [0-9]+ MHz(?:, center2: [0-9]+ MHz)?");
	std::smatch first_channel, second_channel;
	return std::regex_search(first, first_channel, channel) && std::regex_search(second, second_channel, channel) &&
		first_channel[0] == second_channel[0];
}

/** @brief Parse every channel component; unknown channel descriptions reject unsupported input. */
inline RadioChannel radio_channel(const std::string& info) {
	std::smatch match;
	const std::regex pattern(
		"channel [0-9]+ \\(([0-9]+) MHz\\), width: ([0-9]+) MHz, center1: ([0-9]+) MHz(?:, center2: ([0-9]+) MHz)?");
	if (!std::regex_search(info, match, pattern)) {
		throw std::runtime_error("Cannot determine complete AP radio channel");
	}
	return {static_cast<uint32_t>(std::stoul(match[1])), static_cast<uint32_t>(std::stoul(match[3])),
		match[4].matched ? static_cast<uint32_t>(std::stoul(match[4])) : 0,
		static_cast<uint16_t>(std::stoul(match[2]))};
}
/**
 * @brief Require permission to initiate transmission on all occupied 20 MHz channels.
 * A station may join a NO_IR channel while a GO cannot host there. DFS/radar
 * channels need additional driver/CAC support, so automatic hosting excludes them.
 * Never alter regulatory flags to make a candidate appear eligible.
 */
inline bool hosting_allowed(const std::string& channels, const RadioChannel& radio, bool concurrent_no_ir = false) {
	const int first = static_cast<int>(radio.center1) - radio.width / 2 + 10;
	for (int frequency = first; frequency < first + radio.width; frequency += 20) {
		// iw lists regulatory properties beneath the channel, until the next
		// channel/band heading. Include that whole block, not just its first line.
		const std::regex pattern(
			"(?:^|\\n)[ \\t]*\\* " + std::to_string(frequency) + " MHz[^\\n]*(?:\\n[ \\t]+[A-Za-z][^\\n]*)*");
		std::smatch match;
		if (!std::regex_search(channels, match, pattern)) {
			return false;
		}
		auto text = match.str();
		std::transform(text.begin(), text.end(), text.begin(), [](unsigned char value) { return std::tolower(value); });
		if (text.find("disabled") != std::string::npos || text.find("radar") != std::string::npos ||
			(!concurrent_no_ir &&
				(text.find("no ir") != std::string::npos || text.find("no-ir") != std::string::npos))) {
			return false;
		}
	}
	return radio.center2 == 0;
}

/**
 * @brief Check whether this Intel radio can ask the kernel for concurrent hosting.
 * The DKMS module enables the kernel's existing concurrent-GO option and exposes
 * a read-only marker. Distribution kernels with that option enabled also qualify.
 * The patched supplicant checks native GO_CONCURRENT flags and the associated AP
 * before starting a group; the kernel makes the final permission check. Disabled
 * and radar channels remain excluded here, including every part of a wider channel.
 */
inline bool concurrent_hosting_allowed(
	const std::string& interface, const std::string& channels, const RadioChannel& radio) {
	std::error_code error;
	const auto driver = std::filesystem::read_symlink("/sys/class/net/" + interface + "/device/driver", error);
	if (error || driver.filename() != "iwlwifi") {
		return false;
	}
	std::ifstream marker("/sys/module/cfg80211/parameters/mrs_wifi_direct_relax_no_ir");
	std::string enabled;
	marker >> enabled;
	bool active = enabled == "Y" || enabled == "1";
	struct utsname kernel{};
	if (!active && ::uname(&kernel) == 0) {
		std::ifstream config("/boot/config-" + std::string(kernel.release));
		std::string line;
		while (std::getline(config, line)) {
			active |= line == "CONFIG_CFG80211_REG_RELAX_NO_IR=y";
		}
	}
	return active && hosting_allowed(channels, radio, true);
}
}  // namespace mrs_uav_wifi_direct
