// SPDX-License-Identifier: BSD-3-Clause
/** @file radio_discovery.hpp
 * @brief Collect passive Wi-Fi telemetry independently of QUIC and group management.
 */
#pragma once

#include <condition_variable>
#include <mutex>
#include <thread>

#include "radio_metrics.hpp"
#include "wifi_discovery.hpp"

namespace mrs_uav_wifi_direct {
/** @brief A completed radio sample; its timestamp distinguishes fresh publication from fresh measurement. */
struct RadioSnapshot {
	Clock::time_point sampled{};
	bool available{};
	std::string driver, phy, error, discovery_error;
	msg::LinkMetrics ap{empty_link()};
	std::vector<msg::LinkMetrics> stations;
	NearbyUavs nearby;
};
/**
 * @brief Read existing supplicant caches and kernel station statistics on a worker.
 * This observer reads radio state and caches through existing interfaces. The
 * fleet manager handles active discovery when communication is enabled. With an
 * empty fleet, observations depend on advertisements already cached by the host.
 * Separate sockets and short deadlines keep telemetry independent of Wi-Fi setup.
 */
class RadioDiscovery {
	public:
	/** @brief Start a worker whose lifetime ends before the referenced configuration is destroyed. */
	explicit RadioDiscovery(const Config& config) : config_(config), worker_([this]() { run(); }) {}
	/** @brief Wake the worker and wait for any bounded read already in progress. */
	~RadioDiscovery() {
		{
			std::lock_guard<std::mutex> lock(mutex_);
			stop_ = true;
		}
		wake_.notify_all();
		worker_.join();
	}
	RadioDiscovery(const RadioDiscovery&) = delete;
	RadioDiscovery& operator=(const RadioDiscovery&) = delete;
	/** @brief Return a value snapshot; the ROS executor never waits for radio I/O. */
	RadioSnapshot status() const {
		std::lock_guard<std::mutex> lock(mutex_);
		return snapshot_;
	}

	private:
	/** @brief Convert an optional sysfs link into its final component, retaining an empty unknown value. */
	static std::string link_name(const std::string& path) {
		std::error_code error;
		const auto link = std::filesystem::read_symlink(path, error);
		return error ? std::string() : link.filename().string();
	}
	/** @brief Read local AP information and cached UAV advertisements through private, read-only sockets. */
	void observe(RadioSnapshot& sample, const std::vector<msg::LinkMetrics>& stations) {
		WpaControl station(config_.control_directory + "/" + config_.interface, 300);
		station.set_deadline(Clock::now() + 1500ms);
		const auto status = station.request("STATUS");
		std::smatch value;
		if (std::regex_search(status, value, std::regex("(?:^|\\n)ssid=([^\\n]*)"))) {
			sample.ap.ssid = value[1];
		}
		if (std::regex_search(status, value, std::regex("(?:^|\\n)bssid=([0-9A-Fa-f:]{17})"))) {
			sample.ap.mac_address = value[1];
		}
		// A kernel station entry can outlive disassociation briefly. Supplicant
		// state confirms whether this station currently represents the AP link.
		sample.ap.connected = status.find("wpa_state=COMPLETED") != std::string::npos;
		std::transform(sample.ap.mac_address.begin(), sample.ap.mac_address.end(), sample.ap.mac_address.begin(),
			[](unsigned char character) { return std::tolower(character); });
		for (auto station : stations) {
			if (station.mac_address != sample.ap.mac_address) {
				continue;
			}
			station.ssid = sample.ap.ssid;
			station.address = sample.ap.address;
			station.connected = sample.ap.connected;
			station.frequency_mhz = sample.ap.frequency_mhz;
			station.channel_width_mhz = sample.ap.channel_width_mhz;
			sample.ap = std::move(station);
			break;
		}
		std::string control = config_.control_interface;
		if (control.empty()) {
			std::error_code error;
			control = std::filesystem::exists(config_.control_directory + "/p2p-dev-" + config_.interface, error)
				? "p2p-dev-" + config_.interface : config_.interface;
		}
		WpaControl radio(config_.control_directory + "/" + control, 300);
		radio.set_deadline(Clock::now() + 1500ms);
		discover_owners(radio, station, config_, std::nullopt, sample.nearby);
	}
	/** @brief Collect independent measurements; failure of cache access does not discard working driver metrics. */
	RadioSnapshot collect() {
		RadioSnapshot sample;
		sample.sampled = Clock::now();
		sample.ap.interface = config_.interface;
		sample.ap.address = interface_network(config_.interface).address;
		std::vector<msg::LinkMetrics> ap_stations;
		try {
			const auto base = "/sys/class/net/" + config_.interface;
			sample.phy = link_name(base + "/phy80211");
			sample.driver = link_name(base + "/device/driver");
			if (sample.phy.empty()) {
				throw std::runtime_error("No wireless radio for " + config_.interface);
			}
			ap_stations = station_metrics(config_.interface);
			// Interface metadata is obtained directly from the kernel and does not scan.
			radio_query(NL80211_CMD_GET_INTERFACE, NL80211_ATTR_IFINDEX, ::if_nametoindex(config_.interface.c_str()),
				false, [&](nlattr** attrs) {
					if (attrs[NL80211_ATTR_WIPHY_FREQ]) {
						sample.ap.frequency_mhz = nla_get_u32(attrs[NL80211_ATTR_WIPHY_FREQ]);
					}
					if (attrs[NL80211_ATTR_CHANNEL_WIDTH]) {
						const auto width = nla_get_u32(attrs[NL80211_ATTR_CHANNEL_WIDTH]);
						const std::map<uint32_t, uint16_t> widths{{NL80211_CHAN_WIDTH_20_NOHT, 20},
							{NL80211_CHAN_WIDTH_20, 20}, {NL80211_CHAN_WIDTH_40, 40}, {NL80211_CHAN_WIDTH_80, 80},
							{NL80211_CHAN_WIDTH_160, 160}};
						const auto found = widths.find(width);
						if (found != widths.end()) {
							sample.ap.channel_width_mhz = found->second;
						}
					}
				});
			sample.available = true;
			// Discover existing P2P interfaces on this PHY. Interface names alone
			// are insufficient: other radios may have identically prefixed groups.
			for (const auto& entry : std::filesystem::directory_iterator("/sys/class/net")) {
				const auto interface = entry.path().filename().string();
				if (interface == config_.interface || link_name(entry.path().string() + "/phy80211") != sample.phy ||
					!interface_carrier(interface)) {
					continue;
				}
				bool group = false;
				radio_query(NL80211_CMD_GET_INTERFACE, NL80211_ATTR_IFINDEX, ::if_nametoindex(interface.c_str()), false,
					[&](nlattr** attrs) {
						if (attrs[NL80211_ATTR_IFTYPE]) {
							const auto type = nla_get_u32(attrs[NL80211_ATTR_IFTYPE]);
							group = type == NL80211_IFTYPE_P2P_GO || type == NL80211_IFTYPE_P2P_CLIENT;
						}
					});
				if (group) {
					for (auto link : station_metrics(interface)) {
						link.address = interface_network(interface).address;
						sample.stations.push_back(std::move(link));
					}
				}
			}
		} catch (const std::exception& error) {
			sample.error = error.what();
		}
		try {
			observe(sample, ap_stations);
		} catch (const std::exception& error) {
			sample.discovery_error = error.what();
		}
		return sample;
	}
	/** @brief Replace complete snapshots at a bounded rate, stopping promptly between collection cycles. */
	void run() {
		for (;;) {
			auto sample = collect();
			std::unique_lock<std::mutex> lock(mutex_);
			// A missed cache reply retains recent observations until their own age
			// expires. A fresh publication never renews an advertisement's age.
			for (const auto& [address, observation] : snapshot_.nearby) {
				const auto current = sample.nearby.find(address);
				if (Clock::now() - observation.seen <= std::chrono::seconds(config_.peer_timeout) &&
					(current == sample.nearby.end() || current->second.seen < observation.seen)) {
					sample.nearby[address] = observation;
				}
			}
			snapshot_ = std::move(sample);
			// A 10 Hz publication can repeat the latest sample. Polling the radio
			// at most once per second avoids excessive control traffic.
			if (wake_.wait_for(lock, std::chrono::duration<double>(std::max(1.0, config_.discovery_interval)),
					[this]() { return stop_; })) {
				return;
			}
		}
	}
	const Config& config_;
	mutable std::mutex mutex_;
	std::condition_variable wake_;
	bool stop_{};
	RadioSnapshot snapshot_;
	std::thread worker_;
};
}  // namespace mrs_uav_wifi_direct
