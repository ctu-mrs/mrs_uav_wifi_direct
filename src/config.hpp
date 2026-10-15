// SPDX-License-Identifier: BSD-3-Clause
/** @file config.hpp
 * @brief Read YAML settings and the local UAV name.
 */
#pragma once

#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <map>
#include <regex>
#include <set>
#include <stdexcept>

#include "common.hpp"

namespace mrs_uav_wifi_direct {
/** @brief Check that a topic key is one valid ROS name component. */
inline bool token(const std::string& value) {
	return std::regex_match(value, std::regex("[A-Za-z_][A-Za-z_0-9]*"));
}

/** @brief Substitute {hostname} in topics/paths and expand the leading ~/ in TLS paths. */
inline std::string expand(std::string value, const std::string& name) {
	for (size_t pos = 0; (pos = value.find("{hostname}", pos)) != std::string::npos;) {
		value.replace(pos, std::string("{hostname}").size(), name);
		pos += name.size();
	}
	if (value.compare(0, 2, "~/") == 0) {
		const auto home = std::getenv("HOME");
		if (!home) {
			throw std::runtime_error("HOME is needed for ~ paths");
		}
		value.replace(0, 1, home);
	}
	return value;
}

/** @brief Read a YAML setting or use its default when omitted. */
template <typename T>
T option(const YAML::Node& node, const char* key, const T& fallback) {
	return node && node[key] ? node[key].as<T>() : fallback;
}

/** @brief Settings for one shared topic; hz=0 sends every new sample. */
struct TopicConfig {
	std::string key, source, type;
	double hz{};
	bool reliable{};
};
/**
 * @brief Validated runtime settings, loaded before sockets or radio state change.
 * All UAVs use the same fleet list and certificate policy. The local name comes
 * from the hostname parameter, then UAV_NAME, then the system hostname.
 * AP and P2P addressing come from their interfaces' actual host configuration.
 */
struct Config {
	int own{}, port{4567}, direct_dscp{8}, address_timeout{15};
	int election_settle{30}, peer_timeout{12}, switch_hold{5}, host_cooldown{60};
	bool ap{true}, fallback{true}, wifi{true}, port_per_uav{}, automatic_identity{};
	size_t max_message{8388608}, max_pending{16777216};
	std::set<int> numbers;
	std::map<int, std::string> names;
	std::string interface, control_directory, control_interface;
	std::string existing_interface, prefix, cert, key, ca, identity_directory;
	std::vector<TopicConfig> topics;

	/** @brief Return the configured spelling of the UAV name, including a leading zero. */
	std::string name(int number) const {
		const auto entry = names.find(number);
		return entry == names.end() ? uav_name(number) : entry->second;
	}

	/** @brief Use a common port, or add the UAV number for multiple processes on one host. */
	uint16_t listen_port(int number) const {
		return static_cast<uint16_t>(port + (port_per_uav ? number : 0));
	}

	/** @brief Load YAML and reject inconsistent names, bounds, subnet mappings or topics. */
	Config(const std::string& path, std::string local_name) {
		const auto root = YAML::LoadFile(path);
		if (!root.IsMap()) {
			throw std::runtime_error("Configuration must be a YAML mapping");
		}
		if (!root["uav_list"] || !root["uav_list"].IsSequence()) {
			throw std::runtime_error("uav_list must be a list of full UAV names");
		}
		// An empty fleet is the installed default. Stop before identity, topic,
		// certificate or network settings are needed, including on a non-UAV PC.
		if (root["uav_list"].size() == 0) {
			return;
		}
		if (local_name.empty()) {
			const auto environment_name = std::getenv("UAV_NAME");
			if (environment_name) {
				local_name = environment_name;
			}
		}
		if (local_name.empty()) {
			std::array<char, 256> host{};
			if (::gethostname(host.data(), host.size() - 1) != 0) {
				throw std::runtime_error("gethostname failed");
			}
			local_name = std::string(host.data()).substr(0, std::string(host.data()).find('.'));
		}
		std::smatch name_match;
		if (!std::regex_match(local_name, name_match, std::regex("uav([0-9]{1,2})"))) {
			throw std::runtime_error("UAV name must be 'uav' followed by a number from 1 to 99");
		}
		own = std::stoi(name_match[1]);
		for (const auto& item : root["uav_list"]) {
			const auto peer_name = item.as<std::string>();
			std::smatch match;
			if (!std::regex_match(peer_name, match, std::regex("uav([0-9]{1,2})"))) {
				throw std::runtime_error("uav_list entries must be 'uav' followed by a number from 1 to 99");
			}
			const int number = std::stoi(match[1]);
			if (!numbers.insert(number).second) {
				throw std::runtime_error("Duplicate or aliased UAV identity in uav_list");
			}
			names[number] = peer_name;
		}
		if (!numbers.count(own) || names.at(own) != local_name) {
			throw std::runtime_error("The local UAV name must appear in uav_list");
		}
		const auto network = root["network"], wifi_config = root["wifi_direct"], tls = root["tls"];
		port = option(network, "port", 4567);
		port_per_uav = option(network, "port_per_uav", false);
		ap = option(network, "ap_enabled", true);
		fallback = option(network, "ap_fallback", true);
		direct_dscp = option(network, "direct_dscp", 8);
		if (direct_dscp < 0 || direct_dscp > 63) {
			throw std::runtime_error("network.direct_dscp must be an integer from 0 to 63");
		}
		max_message = option<size_t>(network, "max_message_bytes", max_message);
		max_pending = option<size_t>(network, "max_pending_bytes", max_pending);
		if (port < 1 || port > 65535 - (port_per_uav ? *numbers.rbegin() : 0) || max_message < 4 ||
			max_message > 64 * 1024 * 1024 || max_pending < max_message + 65536 || max_pending > 1024 * 1024 * 1024) {
			throw std::runtime_error("Invalid port or queue limits (max_pending >= max_message + 65536 required)");
		}
		for (const auto number : numbers) {
			if (number < 1 || number > 99) {
				throw std::runtime_error("UAV numbers must be 1..99");
			}
		}
		wifi = option(wifi_config, "enabled", true);
		interface = option(wifi_config, "interface", std::string("wlan0"));
		control_directory = option(wifi_config, "control_directory", std::string("/run/wpa_supplicant"));
		control_interface = option(wifi_config, "control_interface", std::string());
		existing_interface = option(wifi_config, "existing_group_interface", std::string());
		election_settle = option(wifi_config, "election_settle_s", 30);
		peer_timeout = option(wifi_config, "peer_timeout_s", 12);
		switch_hold = option(wifi_config, "switch_hold_s", 5);
		host_cooldown = option(wifi_config, "host_cooldown_s", 60);
		if (election_settle < 1 || election_settle > 60 || peer_timeout < 3 || peer_timeout > 120 || switch_hold < 1 ||
			switch_hold > 30 || host_cooldown < 10 || host_cooldown > 600) {
			throw std::runtime_error("Invalid Wi-Fi Direct fleet election timing");
		}
		address_timeout = option(wifi_config, "address_timeout_s", 15);
		if (address_timeout < 1 || address_timeout > 120) {
			throw std::runtime_error("wifi_direct.address_timeout_s must be 1..120");
		}
		if (!ap && !wifi) {
			throw std::runtime_error("Enable AP transport or Wi-Fi Direct");
		}
		const auto name = local_name;
		prefix = expand(option(root, "receive_prefix", std::string("/{hostname}/quic/peers")), name);
		while (!prefix.empty() && prefix.back() == '/') {
			prefix.pop_back();
		}
		if (prefix.empty()) {
			throw std::runtime_error("receive_prefix must not be empty or /");
		}
		cert = expand(option(tls, "certificate", std::string()), name);
		key = expand(option(tls, "private_key", std::string()), name);
		ca = expand(option(tls, "ca_certificate", std::string()), name);
		identity_directory =
			expand(option(tls, "identity_directory", std::string("~/.ros/mrs_uav_wifi_direct/{hostname}")), name);
		if (identity_directory.empty() || cert.empty() != key.empty()) {
			throw std::runtime_error("Set both certificate/private_key or neither, and a nonempty identity_directory");
		}
		automatic_identity = cert.empty() && ca.empty();
		if (cert.empty()) {
			cert = identity_directory + "/" + name + ".crt";
		}
		if (key.empty()) {
			key = identity_directory + "/" + name + ".key";
		}
		for (const auto& file : {cert, key, ca}) {
			if (!file.empty() && !(automatic_identity && file != ca) && !std::filesystem::is_regular_file(file)) {
				throw std::runtime_error("Missing explicitly configured TLS file " + file);
			}
		}
		std::set<std::string> keys;
		const auto shared = root["shared_topics"];
		if (shared && !shared.IsSequence()) {
			throw std::runtime_error("shared_topics must be a list");
		}
		if (shared) {
			for (const auto& entry : shared) {
				TopicConfig topic{entry["key"].as<std::string>(), expand(entry["topic"].as<std::string>(), name),
					entry["message_type"].as<std::string>(), option(entry, "rate_hz", 0.0)};
				const auto reliability = option(entry, "reliability", std::string("best_effort"));
				if (reliability != "best_effort" && reliability != "reliable") {
					throw std::runtime_error("Topic reliability must be best_effort or reliable");
				}
				topic.reliable = reliability == "reliable";
				if (!token(topic.key) || topic.key.size() > 1024 || !keys.insert(topic.key).second ||
					topic.source.empty() || topic.type.empty() || topic.type.size() > 1024 ||
					!std::isfinite(topic.hz) || topic.hz < 0 || topic.hz > 1000000 ||
					(topic.hz > 0 && 1e9 / topic.hz >= static_cast<double>(std::numeric_limits<int64_t>::max()))) {
					throw std::runtime_error("Invalid topic key, type, source or rate_hz");
				}
				topics.push_back(std::move(topic));
			}
		}
		if (topics.size() > 1024) {
			throw std::runtime_error("At most 1024 shared topics are supported");
		}
	}
};

}  // namespace mrs_uav_wifi_direct
