// SPDX-License-Identifier: BSD-3-Clause
/** @file wifi_direct.hpp
 * @brief Create, join, and remove a Wi-Fi Direct group alongside the AP connection.
 */
#pragma once

#include <atomic>
#include <fstream>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "config.hpp"
#include "network_utils.hpp"
#include "radio_lease.hpp"
#include "status.hpp"
#include "wifi_discovery.hpp"
#include "wifi_radio.hpp"
#include "wpa_control.hpp"

namespace mrs_uav_wifi_direct {
/**
 * @brief Maintain a P2P group on the AP's channel while the AP is connected.
 * A worker owns supplicant operations so discovery does not block ROS callbacks.
 * Peers elect an eligible owner; clients discover its current group by UAV name.
 * A group becomes ready only after the existing network manager addresses it. Cleanup removes
 * only the group created here; externally supplied interfaces are merely observed.
 */
class WifiDirect {
	public:
	/** @brief Take the radio lock before starting QUIC or the Wi-Fi worker. */
	WifiDirect(const Config& config, rclcpp::Logger logger) : config_(config), logger_(logger) {
		if (config_.wifi && config_.existing_interface.empty()) {
			lease_ = std::make_unique<RadioLease>(config_.interface);
			config_.p2p_subnet.prepare(config_.interface, lease_->previous_group());
		}
	}
	/** @brief Stop the worker and remove its owned group before configuration is destroyed. */
	~WifiDirect() {
		stop_ = true;
		if (thread_.joinable()) {
			thread_.join();
		}
	}
	/** @brief Start automatic management unless an external group interface is configured. */
	void start() {
		if (config_.wifi && config_.existing_interface.empty() && !thread_.joinable()) {
			thread_ = std::thread([this]() { run(); });
		}
	}
	/** @brief Return the group interface once it has a working link and an assigned IPv4 address. */
	std::string ready_interface() {
		std::string name;
		{
			std::lock_guard<std::mutex> lock(mutex_);
			name = config_.existing_interface.empty() ? group_ : config_.existing_interface;
		}
		const auto network = interface_network(name);
		return config_.wifi && interface_carrier(name) && network &&
				(!config_.existing_interface.empty() || config_.p2p_subnet.matches(network, config_.own))
			? name
			: "";
	}

	/** @brief Publish a consistent capability/group snapshot to the ROS maintenance callback. */
	FleetStatus fleet_status() {
		std::lock_guard<std::mutex> lock(mutex_);
		return fleet_status_;
	}
	/** @brief Copy group/discovery diagnostics without issuing Wi-Fi commands. */
	WifiStatus status() {
		std::lock_guard<std::mutex> lock(mutex_);
		return {fleet_status_, discovered_, nearby_,
			config_.existing_interface.empty() ? group_ : config_.existing_interface, last_error_};
	}
	/** @brief Replace authenticated observations; the radio worker makes all policy decisions. */
	void observe_peers(FleetPeers peers) {
		std::lock_guard<std::mutex> lock(mutex_);
		peers_ = std::move(peers);
	}

	private:
	/** @brief Retry failed group setup after a short wait that can be interrupted by shutdown. */
	void run() {
		while (!stop_) {
			int wait_steps = 10;
			try {
				manage_group();
			} catch (const std::exception& error) {
				{
					std::lock_guard<std::mutex> lock(mutex_);
					fleet_status_ = {};
					discovered_.clear();
					last_error_ = error.what();
				}
				RCLCPP_WARN(logger_, "Wi-Fi Direct unavailable: %s; retrying in 10 seconds", error.what());
				wait_steps = 50;
			}
			// Let asynchronous driver teardown finish before changing the P2P role.
			for (int i = 0; i < wait_steps && !stop_; ++i) {
				std::this_thread::sleep_for(200ms);
			}
		}
	}
	/**
	 * @brief Use the AP's full channel when present, otherwise discover/form a standalone group.
	 * WPS-PBC enrolls the radio peer; subsequent mutual TLS authenticates application
	 * identities. AP channel changes trigger removal/reformation of this owned group.
	 * The existing network manager continues to control the AP connection and routes.
	 */
	void manage_group() {
		// Every attempt starts without IP connectivity on temporary WPS clients.
		config_.p2p_subnet.prepare(config_.interface, lease_->previous_group());
		std::ifstream phy_file("/sys/class/net/" + config_.interface + "/phy80211/name");
		std::string phy;
		phy_file >> phy;
		if (phy.empty()) {
			throw std::runtime_error("Cannot identify the Wi-Fi radio");
		}
		const auto capabilities = command({"iw", "phy", phy, "info"});
		std::error_code driver_error;
		const auto driver =
			std::filesystem::read_symlink("/sys/class/net/" + config_.interface + "/device/driver", driver_error)
				.filename();
		const bool full_mac = !driver_error && driver == "brcmfmac";
		const bool client_capable = concurrent_combination(capabilities, false);
		const bool owner_capable = concurrent_combination(capabilities, true);
		if (!client_capable && !owner_capable) {
			throw std::runtime_error("Driver does not advertise managed + P2P interface concurrency");
		}
		WpaControl ap(config_.control_directory + "/" + config_.interface);
		WpaControl ap_monitor(config_.control_directory + "/" + config_.interface);
		ap_monitor.ok("ATTACH");
		auto status = ap.request("STATUS");
		std::smatch freq;
		bool ap_present = status.find("wpa_state=COMPLETED") != std::string::npos &&
			std::regex_search(status, freq, std::regex("(?:^|\\n)freq=([0-9]+)"));
		// A wider AP and a 20 MHz GO can create separate firmware channel contexts
		// even with the same primary frequency. Request the AP's width as well.
		auto ap_radio = ap_present ? command({"iw", "dev", config_.interface, "info"}) : "";
		const auto channels = command({"iw", "phy", phy, "channels"});
		const auto standalone = standalone_channel(channels);
		FleetStatus local;
		local.ap_present = ap_present;
		local.radio = ap_present   ? radio_channel(ap_radio)
			: standalone.frequency ? standalone
								   : RadioChannel{2412, 2412, 0, 20};
		std::string frequency = std::to_string(local.radio.frequency);
		if (ap_present) {
			last_ap_frequency_ = local.radio.frequency;
		}
		local.can_join = client_capable;
		bool host_allowed =
			owner_capable && (ap_present ? hosting_allowed(channels, local.radio) : standalone.frequency != 0);
		if (ap_present && owner_capable && !host_allowed) {
			// Intel concurrent hosting needs both kernel and supplicant support.
			// The supplicant checks the AP and native channel flags during formation.
			host_allowed = ap.request("GET mrs_wifi_direct_radio_support") == "3" &&
				concurrent_hosting_allowed(config_.interface, channels, local.radio);
		}
		if (cooldown_radio_.frequency && !(cooldown_radio_ == local.radio)) {
			host_retry_ = Clock::time_point{};
		}
		cooldown_radio_ = local.radio;
		local.can_host = host_allowed && Clock::now() >= host_retry_;
		RCLCPP_INFO(logger_, "Fleet radio eligibility: host=%s client=%s on %s MHz", local.can_host ? "yes" : "no",
			local.can_join ? "yes" : "no", frequency.c_str());
		const int ap_width = local.radio.width;
		std::string channel_options;
		if (ap_width == 40) {
			channel_options = " ht40 max_oper_chwidth=40";
		} else if (ap_width == 80 || ap_width == 160) {
			channel_options = " vht max_oper_chwidth=" + std::to_string(ap_width);
		} else if (ap_width != 20) {
			throw std::runtime_error("AP channel width is not supported for concurrent P2P");
		}
		std::string control = config_.control_interface;
		if (control.empty()) {
			control = config_.interface;
			std::error_code status_error;
			// The one-time setup grants direct group access to the existing socket
			// directory; no intermediary process or executable capability is used.
			if (std::filesystem::exists(config_.control_directory + "/p2p-dev-" + config_.interface, status_error) ||
				status_error) {
				control = "p2p-dev-" + config_.interface;
			}
		}
		WpaControl request(config_.control_directory + "/" + control);
		WpaControl monitor(config_.control_directory + "/" + control);
		monitor.ok("ATTACH");
		if (const auto previous = lease_->previous_group(); !previous.empty() && previous != config_.interface) {
			std::ifstream old_phy_file("/sys/class/net/" + previous + "/phy80211/name");
			std::string old_phy;
			old_phy_file >> old_phy;
			const auto old_role = command({"iw", "dev", previous, "info"});
			if (old_phy == phy &&
				(old_role.find("type P2P-GO") != std::string::npos ||
					old_role.find("type P2P-client") != std::string::npos)) {
				request.ok("P2P_GROUP_REMOVE " + previous);
				RCLCPP_INFO(logger_, "Recovered this node's group after an interrupted run: %s", previous.c_str());
			}
		}
		lease_->forget();
		if (full_mac) {
			const auto interfaces = control_interfaces(request);
			if (std::all_of(interfaces.begin(), interfaces.end(),
					[&](const auto& interface) { return interface == config_.interface || interface == control; })) {
				// Full-MAC firmware can retain discovery state across client/owner
				// transitions. Reset P2P state while no group exists. The primary
				// station stays associated; externally managed groups prevent reset.
				request.ok("P2P_SET disabled 1");
				request.ok("P2P_SET disabled 0");
			}
		}
		// Prefer the existing station connection when supplicant resolves concurrency.
		request.ok("P2P_SET conc_pref sta");
		// NO_IR prohibits creating a GO, but permits joining an existing GO.
		// Include driver-permitted client channels without changing regulation.
		request.ok("SET p2p_add_cli_chan 1");
		const auto channel = ap_listen_channel(std::stoi(frequency));
		request.ok("P2P_SET listen_channel " + std::to_string(channel.first) + " " + std::to_string(channel.second));
		request.ok("SET device_name " + config_.name(config_.own));
		// Native discovery can coordinate before an IP path exists. This bounded
		// WPS field advertises channel settings and AP constraints; TLS verifies identity.
		request.ok("SET model_name " + radio_model(local.radio, ap_present));
		std::string created;
		bool enrolling = false, hosting = false, host_requested = false;
		int owner = 0;
		DiscoveredOwners discovered;
		DiscoveredOwner joining_owner;
		const auto began = Clock::now();
		// Stagger autonomous startup when no IP coordination path exists. Once a GO
		// is visible, all compatible clients prefer it over any idle candidate.
		const auto rank = std::distance(config_.numbers.begin(), config_.numbers.find(config_.own));
		const auto settle_until = began + std::chrono::seconds(config_.election_settle) +
			std::chrono::milliseconds(std::min<decltype(rank)>(rank, 32) * (config_.ap && ap_present ? 250 : 3000));
		int proposed = 0;
		auto proposal_since = began;
		int isolated_peer = 0;
		auto isolated_since = began;
		auto owner_lease = began;
		auto publish_status = [&]() {
			std::lock_guard<std::mutex> lock(mutex_);
			fleet_status_ = local;
			last_error_.clear();
		};
		publish_status();
		const auto original_interfaces = control_interfaces(request);
		std::unique_ptr<WpaControl> pending_client;
		auto next_pending_check = Clock::time_point{};
		auto next_pbc = Clock::time_point{};
		auto enrollment_until = Clock::time_point{};
		auto empty_group_since = Clock::now();
		auto address_deadline = Clock::time_point{};
		bool addressed = false, address_requested = false;
		std::unique_ptr<WpaControl> group_control;
		std::unique_ptr<WpaControl> group_monitor;
		// With an AP, pin the first scan and listening, and delay subsequent social
		// searches beyond the two-second discovery lifetime. Without an AP, cycle
		// individual supported channels; its arrival reforms a pinned group.
		// Scan one supported channel at a time even without an AP. Some Full-MAC
		// firmware rejects unrestricted multi-band discovery after disassociation.
		// Start with the last AP channel so a disconnected client can promptly
		// rejoin its still-connected peers, then visit legal social/5 GHz channels.
		const auto preferred_frequency = last_owner_frequency_ ? last_owner_frequency_
			: last_ap_frequency_							   ? last_ap_frequency_
															   : local.radio.frequency;
		const auto scan_channels = discovery_channels(channels, preferred_frequency);
		const auto bootstrap_primary = scan_channels.front();
		// Full-MAC native discovery can hide GOs or leave the firmware unable
		// to accept clients after becoming an owner. Use its station beacon
		// scans. Other drivers alternate beacon scans and native P2P discovery.
		size_t scan_index = 0, isolated_index = 0, discovery_attempt = 0;
		auto discover = [&](bool isolated = false) {
			// Revisit a remembered owner/AP channel between other targets so
			// cleanup and retry cannot lose a reachable group for a band rotation.
			// Revisit both usual startup bands between the less common scan
			// targets. A GO may appear just after our first scan; a full-band
			// rotation must not hide it for minutes while two isolated groups wait.
			const uint32_t other_band = bootstrap_primary >= 5000 ? 2412 : 5180;
			const bool other_supported =
				std::find(scan_channels.begin(), scan_channels.end(), other_band) != scan_channels.end();
			const auto bootstrap = scan_index % 3 == 0	 ? bootstrap_primary
				: scan_index % 3 == 1 && other_supported ? other_band
														 : scan_channels.at((scan_index / 3) % scan_channels.size());
			const bool current_channel = isolated && (isolated_index++ % 2 == 0);
			auto target = ap_present || current_channel ? frequency : std::to_string(bootstrap);
			const uint32_t cross_band = local.radio.frequency >= 5000 ? 2412 : 5180;
			if (isolated && !ap_present && isolated_index % 4 == 1 &&
				std::find(scan_channels.begin(), scan_channels.end(), cross_band) != scan_channels.end()) {
				// A vanished AP can leave the remembered channel different from
				// the new standalone GO's default. Try the other startup band
				// first; retain same-channel and general scans on later attempts.
				target = std::to_string(cross_band);
			}
			if (!ap_present && !hosting && proposed && discovered.count(proposed)) {
				// Keep a discovered owner's beacon fresh throughout the election
				// settle/hold period. Rotating away would let its three-second
				// join observation expire before CONNECT is allowed to begin.
				target = std::to_string(discovered.at(proposed).radio.frequency);
			}
			if (!ap_present && !current_channel) {
				++scan_index;
			}
			if (!full_mac && discovery_attempt++ % 2 == 0) {
				request.ok("P2P_FIND 2 freq=" + target + " delay=10000");
			} else if ((created.empty() && !enrolling) || isolated) {
				// A remembered group SSID permits directed probing when firmware
				// filters wildcard results. Never broaden the selected channel.
				std::string directed;
				if (proposed && proposed != config_.own && discovered.count(proposed) &&
					discovered.at(proposed).radio.frequency == std::stoul(target)) {
					directed = discovered.at(proposed).ssid;
				}
				ap.request("SCAN freq=" + target + (directed.empty() ? "" : " ssid " + directed));
			}
		};
		auto observe_ap_change = [&]() {
			if (created.empty() || !addressed) {
				return;
			}
			const bool associated = ap.request("STATUS").find("wpa_state=COMPLETED") != std::string::npos;
			if (!ap_present && associated) {
				const auto current_radio = command({"iw", "dev", config_.interface, "info"});
				if (radio_channel(current_radio) == local.radio) {
					// Returning on the existing group channel needs no role change or
					// new enrollment. Resume AP pinning and preserve working sessions.
					ap_radio = current_radio;
					ap_present = true;
					local.ap_present = true;
					last_ap_frequency_ = local.radio.frequency;
					request.ok("SET model_name " + radio_model(local.radio, true));
					publish_status();
					RCLCPP_INFO(logger_, "AP returned on the direct group channel; retaining Wi-Fi Direct group");
				}
			} else if (ap_present && !associated) {
				if (hosting && !hosting_allowed(command({"iw", "phy", phy, "channels"}), local.radio)) {
					// A concurrent NO_IR exception needs an associated AP. Once it
					// disappears, reform on a legal standalone channel instead.
					throw std::runtime_error("AP disconnected; this GO channel requires its AP permission");
				}
				// An AP outage does not invalidate an already working direct link.
				// Retain it and advertise the relaxed policy. A lost GO/interface is
				// still handled by its normal events and heartbeat expiry below.
				ap_present = false;
				local.ap_present = false;
				request.ok("SET model_name " + radio_model(local.radio, false));
				publish_status();
				RCLCPP_INFO(logger_, "AP disconnected; retaining the established Wi-Fi Direct group");
			}
		};
		auto cleanup = [&]() {
			if (!full_mac) {
				try {
					request.ok("P2P_STOP_FIND");
				} catch (...) {
				}
			}
			{
				std::lock_guard<std::mutex> lock(mutex_);
				group_.clear();
				local.role = FleetRole::idle;
				local.owner = 0;
				local.members = 0;
				local.ready = false;
				fleet_status_ = local;
			}
			if (enrolling || (host_requested && created.empty())) {
				try {
					// Pending enrollment/formation may already own a temporary interface.
					// Cancel explicitly so shutdown/retries never leave a radio slot busy.
					request.ok("P2P_CANCEL");
				} catch (...) {
				}
			}
			if (host_requested && created.empty()) {
				// A driver may create a GO but omit GROUP-STARTED. Reclaim only newly
				// created GO interfaces after our successful GROUP_ADD, never the AP
				// or an interface that existed before this formation attempt.
				try {
					for (const auto& interface : control_interfaces(request)) {
						if (!original_interfaces.count(interface) && interface != config_.interface &&
							command({"iw", "dev", interface, "info"}).find("type P2P-GO") != std::string::npos) {
							request.ok("P2P_GROUP_REMOVE " + interface);
						}
					}
				} catch (...) {
				}
			}
			if (!created.empty()) {
				try {
					request.ok("P2P_GROUP_REMOVE " + created);
					lease_->forget();
				} catch (...) {
				}
			}
		};
		try {
			discover();
			auto next_scan = Clock::now() + 4s;
			auto next_join = Clock::time_point{};
			auto next_check = Clock::now() + 5s;
			auto next_peer_check = Clock::now() + 2s;
			auto next_isolated_scan = Clock::now() + 5s;
			// Joining is allowed only after a fresh beacon identified a formed GO,
			// never merely a similarly named idle device or a stale cached address.
			auto join_owner = [&](const DiscoveredOwner& peer) {
				joining_owner = peer;
				if (peer.radio.frequency) {
					last_owner_frequency_ = peer.radio.frequency;
				}
				if (!ap_present && peer.radio.frequency) {
					frequency = std::to_string(peer.radio.frequency);
				}
				RCLCPP_INFO(logger_, "Joining elected Wi-Fi Direct owner %s (%s) on %s MHz",
					config_.name(owner).c_str(), peer.device.c_str(), frequency.c_str());
				std::string options = " freq=" + frequency + channel_options;
				if (!ap_present) {
					options = peer.radio.frequency ? " freq=" + frequency : "";
					if (peer.radio.width == 40) {
						options += " ht40 max_oper_chwidth=40";
					} else if (peer.radio.width >= 80) {
						options += " vht max_oper_chwidth=" + std::to_string(peer.radio.width);
					}
				}
				// An exact beacon SSID avoids Broadcom's wildcard-P2P scan path,
				// whose driver rejects an advertised non-social listen channel.
				// The scan/association still uses only the pinned operating channel.
				if (!peer.ssid.empty()) {
					options += " ssid=" + peer.ssid;
				}
				if (!full_mac) {
					request.ok("P2P_STOP_FIND");
				}
				request.ok("P2P_CONNECT " + peer.device + " pbc join" + options);
				enrolling = true;
				local.owner = owner;
				local.role = FleetRole::joining;
				publish_status();
				// WPS may take two minutes; never issue a second concurrent CONNECT.
				next_join = Clock::now() + 150s;
				next_scan = next_join;
			};
			auto refresh_peers = [&]() {
				const std::optional<RadioChannel> pinned =
					ap_present ? std::optional<RadioChannel>(local.radio) : std::nullopt;
				NearbyUavs nearby;
				for (const auto& [peer, observation] : discover_owners(request, ap, config_, pinned, nearby)) {
					const auto failed = failed_groups_.find(peer);
					if (failed != failed_groups_.end()) {
						// A live application does not prove that its group can be joined.
						// Give an alternative owner time to form; a fresh SSID or the
						// cooldown expiry permits another attempt at this owner.
						if (Clock::now() < failed->second.second &&
							(failed->second.first.empty() || observation.ssid.empty() ||
								failed->second.first == observation.ssid)) {
							discovered.erase(peer);
							continue;
						}
						failed_groups_.erase(failed);
					}
					if (!discovered.count(peer)) {
						RCLCPP_INFO(logger_, "Discovered Wi-Fi Direct owner %s on %u MHz", config_.name(peer).c_str(),
							observation.radio.frequency);
					}
					discovered[peer] = observation;
				}
				std::lock_guard<std::mutex> lock(mutex_);
				discovered_ = discovered;
				for (auto& [device, observation] : nearby) {
					nearby_[device] = std::move(observation);
				}
				for (auto entry = nearby_.begin(); entry != nearby_.end();) {
					if (Clock::now() - entry->second.seen > std::chrono::seconds(config_.peer_timeout)) {
						entry = nearby_.erase(entry);
					} else {
						++entry;
					}
				}
			};
			while (!stop_) {
				if (!created.empty() && !addressed) {
					if (!address_requested) {
						const auto state = group_control->request("STATUS");
						// Routers can start before this node. Keep their IP traffic off
						// the temporary WPS association until WPA2 is fully established.
						if (state.find("wpa_state=COMPLETED\n") != std::string::npos &&
							(state.find("key_mgmt=WPA2-PSK\n") != std::string::npos ||
								state.find("key_mgmt=WPA-PSK\n") != std::string::npos)) {
							config_.p2p_subnet.activate(config_.own);
							address_requested = true;
						}
					}
					// The existing network manager applies the address to the completed group.
					const auto network = interface_network(created);
					addressed = address_requested && static_cast<bool>(network);
					if (addressed) {
						if (!config_.p2p_subnet.matches(network, config_.own)) {
							throw std::runtime_error(
								"P2P address differs from network.p2p_subnet; reinstall the support package");
						}
						for (const auto peer : config_.numbers) {
							network.peer_address(config_.own, peer);
						}
						RCLCPP_INFO(logger_, "Using host-configured P2P address %s on %s", network.address.c_str(),
							created.c_str());
						std::lock_guard<std::mutex> lock(mutex_);
						group_ = created;
						owner_lease = Clock::now() + std::chrono::seconds(std::max(config_.peer_timeout, 12));
					}
					local.ready = addressed;
					publish_status();
					if (!addressed && Clock::now() >= address_deadline) {
						throw std::runtime_error(
							"P2P address was not assigned; check the support package's network profile");
					}
				}
				if (enrolling && !pending_client && Clock::now() >= next_pending_check) {
					// GROUP-STARTED arrives after WPS. Its temporary client already
					// scans during WPS, so constrain that new interface immediately.
					for (const auto& interface : control_interfaces(request)) {
						if (original_interfaces.count(interface) || interface == config_.interface) {
							continue;
						}
						const auto radio = command({"iw", "dev", interface, "info"});
						if (radio.find("type P2P-client") == std::string::npos) {
							continue;
						}
						pending_client = std::make_unique<WpaControl>(config_.control_directory + "/" + interface);
						lease_->remember(interface);
						// The discovered GO already fixes the enrollment channel, even
						// without an AP. Unrestricted WPS retry scans are unnecessary
						// and some drivers reject them with EINVAL after disassociation.
						pending_client->ok("SET freq_list " + frequency);
						// Keep only the temporary P2P client awake for association/WPS.
						// Primary station power saving remains under its existing manager.
						pending_client->request("P2P_SET ps 0");
						break;
					}
					next_pending_check = Clock::now() + 1s;
				}
				if (enrolling && Clock::now() >= next_join) {
					throw std::runtime_error("P2P enrollment timed out; cancelling the pending client interface");
				}
				const auto ap_event = ap_monitor.event(0);
				if (!enrolling && ap_event.find("CTRL-EVENT-SCAN-RESULTS") != std::string::npos) {
					// Scans complete asynchronously. An isolated GO otherwise checks
					// peers every ten seconds, after the fresh-beacon window expired.
					refresh_peers();
					next_peer_check = Clock::now() + 2s;
				}
				if (ap_event.find("CTRL-EVENT-DISCONNECTED") != std::string::npos ||
					ap_event.find("CTRL-EVENT-CONNECTED") != std::string::npos ||
					ap_event.find("CTRL-EVENT-ASSOC-REJECT") != std::string::npos) {
					observe_ap_change();
					const auto current = ap.request("STATUS");
					if ((current.find("wpa_state=COMPLETED") != std::string::npos) != ap_present) {
						throw std::runtime_error("AP availability changed; updating this node's P2P channel policy");
					}
				}
				auto event = monitor.event(200);
				if (group_monitor) {
					event += group_monitor->event(0);
				}
				std::smatch match;
				if (std::regex_search(
						event, match, std::regex("P2P-GROUP-STARTED ([^ ]+) (GO|client).*freq=([0-9]+)")) &&
					(enrolling || (hosting && host_requested && created.empty()))) {
					const std::string event_interface = match[1];
					if (event_interface == config_.interface || original_interfaces.count(event_interface)) {
						// Unrelated/pre-existing groups are never owned or removed by us.
						throw std::runtime_error("Supplicant reported a pre-existing interface");
					}
					created = event_interface;
					lease_->remember(created);
					address_deadline = Clock::now() + std::chrono::seconds(config_.address_timeout);
					addressed = false;
					enrolling = false;
					pending_client.reset();
					if (created == config_.interface || ((ap_present || hosting) && match[3] != frequency) ||
						(match[2] == "GO") != hosting) {
						throw std::runtime_error(
							"Supplicant formed a group with an unexpected interface, role or channel");
					}
					// Confirm the driver honored the requested width and center channel.
					// Equal primary frequencies alone can still create separate contexts.
					const auto group_radio = command({"iw", "dev", created, "info"});
					if (ap_present && !same_radio_channel(ap_radio, group_radio)) {
						throw std::runtime_error(
							"P2P group does not match the AP channel width/center; using AP fallback");
					}
					if (!ap_present) {
						local.radio = radio_channel(group_radio);
						frequency = std::to_string(local.radio.frequency);
						request.ok("SET model_name " + radio_model(local.radio, false));
					}
					if (!hosting) {
						std::smatch group_ssid;
						if (std::regex_search(
								event, group_ssid, std::regex("ssid=\"(DIRECT-[A-Za-z0-9_.-]{2,25})\""))) {
							joining_owner.ssid = encode_ssid(group_ssid[1]);
						}
					}

					group_control = std::make_unique<WpaControl>(config_.control_directory + "/" + created);
					group_monitor = std::make_unique<WpaControl>(config_.control_directory + "/" + created);
					group_monitor->ok("ATTACH");
					// Some drivers cannot change P2P power saving through the supplicant,
					// including Intel GO interfaces. The driver may also lack readback.
					// This tuning is optional: keep an otherwise valid group operational.
					// Request it only on our group, preserving the managed AP settings.
					if (group_control->request("P2P_SET ps 0").compare(0, 2, "OK") != 0) {
						RCLCPP_WARN(logger_, "P2P power-saving control unsupported; retaining the driver default on %s",
							created.c_str());
					}
					local.role = hosting ? FleetRole::hosting : FleetRole::client;
					local.owner = owner;
					local.ready = false;
					// Initial listener retries plus TLS startup may exceed a very short
					// configured lease. Give the first handshake a bounded grace period.
					owner_lease = Clock::now() + std::chrono::seconds(std::max(config_.peer_timeout, 12));
					publish_status();
					RCLCPP_INFO(logger_, "Wi-Fi Direct %s on %s, frequency %s MHz; waiting for host-configured IPv4",
						hosting ? "group owner" : "client", created.c_str(), frequency.c_str());
					if (hosting) {
						// Full-MAC owners need an uninterrupted enrollment window.
						// Other radios can find and merge their empty groups meanwhile.
						next_isolated_scan = Clock::now() + (full_mac ? 150s : 5s);
						group_control->ok("WPS_PBC");
						next_pbc = Clock::now() + 120s;
						// A short same-channel discovery initializes GO discovery state on
						// full-MAC drivers. Do it once, before clients enroll, then keep the
						// owner free of recurring scans throughout normal communication.
						discover();
					}
				}
				if (event.find("P2P-GROUP-REMOVED " + created) != std::string::npos && !created.empty()) {
					created.clear();
					throw std::runtime_error("Wi-Fi Direct group was removed");
				}
				if (event.find("P2P-DEVICE-FOUND") != std::string::npos && !enrolling) {
					refresh_peers();
					next_peer_check = Clock::now() + 2s;
				}
				if (group_monitor && !hosting && event.find("CTRL-EVENT-DISCONNECTED") != std::string::npos) {
					throw std::runtime_error("Elected owner disconnected; re-electing the fleet owner");
				}
				// Formation failures contain no credentials; log their reason for deployment diagnosis.
				if (event.find("P2P-GROUP-FORMATION-FAILURE") != std::string::npos ||
					event.find("P2P-GO-NEG-FAILURE") != std::string::npos ||
					event.find("WPS-FAIL") != std::string::npos || event.find("WPS-TIMEOUT") != std::string::npos) {
					// Combined events can include GROUP-STARTED with a password.
					// Log a fixed reason, and cancel before another client is created.
					if (!hosting || created.empty()) {
						throw std::runtime_error("P2P enrollment failed; cancelling before retry");
					}
					next_pbc = Clock::time_point{};
				}
				if (event.find("WPS-SUCCESS") != std::string::npos && hosting) {
					next_pbc = Clock::now() + 5s;
				}
				if (event.find("P2P-PROV-DISC-PBC-REQ") != std::string::npos && group_control && hosting) {
					// Provisioning precedes STA-FIRST visibility. Stop discovery now,
					// even when the existing WPS window is still open, and reserve the
					// radio for enrollment. Associated stations subsequently suppress scans.
					std::smatch peer_name;
					if (std::regex_search(event, peer_name, std::regex("name='(uav[0-9]{1,2})'")) &&
						config_.numbers.count(std::stoi(std::string(peer_name[1]).substr(3)))) {
						next_isolated_scan = Clock::now() + 150s;
						enrollment_until = Clock::now() + 30s;
						if (!full_mac) {
							request.ok("P2P_STOP_FIND");
						}
						if (Clock::now() >= next_pbc) {
							group_control->ok("WPS_PBC");
							next_pbc = Clock::now() + 120s;
						}
					}
				}
				if (Clock::now() >= next_check) {
					observe_ap_change();
					status = ap.request("STATUS");
					const bool associated = status.find("wpa_state=COMPLETED") != std::string::npos;
					if (associated != ap_present ||
						(ap_present && status.find("freq=" + frequency + "\n") == std::string::npos)) {
						throw std::runtime_error(
							"Infrastructure association/channel changed; reforming the direct group");
					}
					// APs can change their bandwidth without changing the primary
					// frequency. Keep both radio contexts equal throughout operation.
					if (ap_present &&
						(!same_radio_channel(ap_radio, command({"iw", "dev", config_.interface, "info"})) ||
							(!created.empty() &&
								!same_radio_channel(ap_radio, command({"iw", "dev", created, "info"}))))) {
						throw std::runtime_error("Radio channel width/center changed; reforming the direct group");
					}
					next_check = Clock::now() + 5s;
				}
				if (!enrolling && Clock::now() >= next_peer_check) {
					refresh_peers();
					next_peer_check = Clock::now() + (created.empty() ? 2s : 10s);
				}
				FleetPeers reports;
				{
					std::lock_guard<std::mutex> lock(mutex_);
					reports = peers_;
				}
				const auto now = Clock::now();
				// Some radios cannot receive another band's beacons while hosting.
				// Two empty groups can then remain invisible to one another. Release
				// an unused standalone group periodically so normal discovery can run
				// with no GO channel context. Keep clients and WPS enrollment intact.
				if (hosting && addressed && !ap_present && local.members == 1 && config_.numbers.size() > 1) {
					const auto station = group_control->request("STA-FIRST");
					if (now < enrollment_until ||
						std::regex_search(station, std::regex("^[0-9A-Fa-f:]{17}(?:\\n|$)"))) {
						empty_group_since = now;
					} else if (now - empty_group_since >= (full_mac ? 150s : 30s)) {
						host_retry_ = now + std::chrono::seconds(config_.host_cooldown);
						RCLCPP_INFO(logger_, "Empty standalone Wi-Fi Direct group restarting discovery");
						break;
					}
				} else {
					empty_group_since = now;
				}
				for (const auto& [peer, failure] : failed_groups_) {
					const auto report = reports.find(peer);
					if (now < failure.second && report != reports.end() &&
						report->second.status.role == FleetRole::hosting) {
						// AP heartbeats may remain healthy while P2P enrollment fails.
						reports.erase(report);
					}
				}
				if (hosting && group_control && now >= next_pbc) {
					// Explicit BSSID joins need not send provisioning discovery.
					// Renew an expired enrollment window so late/returning UAVs can
					// join without a ROS restart. Existing associations stay intact;
					// mutual TLS still restricts topic access to configured identities.
					const bool opened = group_control->request("WPS_PBC").compare(0, 2, "OK") == 0;
					next_pbc = now + (opened ? 120s : 5s);
				}
				local.can_host = host_allowed && now >= host_retry_;
				local.members = 0;
				if (hosting && addressed) {
					local.members = 1;
					for (const auto& [peer, observation] : reports) {
						if (observation.status.role == FleetRole::client && observation.status.ready &&
							observation.status.owner == config_.own &&
							fleet_channel_compatible(local, observation.status) &&
							now - observation.seen <= std::chrono::seconds(config_.peer_timeout)) {
							++local.members;
						}
					}
				}
				publish_status();
				const auto report = reports.find(owner);
				if (!hosting && owner && report != reports.end() &&
					fleet_channel_compatible(local, report->second.status) &&
					report->second.status.role == FleetRole::hosting && report->second.status.ready &&
					now - report->second.seen <= std::chrono::seconds(config_.peer_timeout)) {
					owner_lease = report->second.seen + std::chrono::seconds(config_.peer_timeout);
				}
				int selected = elect_owner(
					config_.own, local, reports, discovered, now, std::chrono::seconds(config_.peer_timeout));
				int alternative = 0;
				if (hosting && addressed && local.members == 1 && selected == config_.own) {
					for (const auto& [peer, hint] : discovered) {
						// Preserve the observation across alternating isolated scans;
						// require a newly refreshed beacon before actually yielding.
						if (now - hint.seen <= std::chrono::seconds(std::max(30, config_.peer_timeout))) {
							alternative = peer;
							break;
						}
					}
				}
				if (alternative != isolated_peer) {
					isolated_peer = alternative;
					isolated_since = now;
				}
				if (isolated_peer && now - discovered.at(isolated_peer).seen <= 3s &&
					now - isolated_since >=
						std::chrono::seconds(std::max(10, config_.peer_timeout + 2 * config_.switch_hold))) {
					// Discovery can be asymmetric: one isolated GO may hear another
					// whose Full-MAC firmware never discovers it. Give the normal
					// identity election time to converge first, then yield an empty
					// group so the reachable UAVs can join one group. Discovery has
					// already checked compatibility with our primary AP channel.
					// An associated/enrolling station always takes precedence.
					const auto station = group_control->request("STA-FIRST");
					if (now >= enrollment_until &&
						!std::regex_search(station, std::regex("^[0-9A-Fa-f:]{17}(?:\\n|$)"))) {
						last_owner_frequency_ = discovered.at(isolated_peer).radio.frequency;
						host_retry_ = now + std::chrono::seconds(config_.host_cooldown);
						RCLCPP_INFO(logger_, "Empty Wi-Fi Direct group yielding to reachable owner %s",
							config_.name(isolated_peer).c_str());
						break;
					}
					isolated_since = now;
				}
				// Allow the first direct QUIC handshake to establish the owner lease.
				// Subsequently a live interface alone cannot keep a vanished host alive.
				if (!hosting && !created.empty() && now < owner_lease && (selected == 0 || selected == config_.own)) {
					selected = owner;
				}
				if (!hosting && !created.empty() && now >= owner_lease) {
					failed_groups_[owner] = {joining_owner.ssid, now + std::chrono::seconds(config_.host_cooldown)};
					throw std::runtime_error("Fleet owner heartbeat expired; re-electing");
				}
				if (selected != proposed) {
					proposed = selected;
					proposal_since = now;
					if (selected && selected != config_.own && created.empty() && !enrolling) {
						// Renew the beacon after the hold period so CONNECT always uses
						// a fresh observation, even when the original scan has aged out.
						next_scan = std::min(next_scan, now + std::chrono::seconds(config_.switch_hold));
					}
				}
				if (!enrolling && now - proposal_since >= std::chrono::seconds(config_.switch_hold)) {
					if (!created.empty() && selected && selected != owner) {
						if (hosting && group_control && local.members == 1 &&
							(now < enrollment_until ||
								std::regex_search(
									group_control->request("STA-FIRST"), std::regex("^[0-9A-Fa-f:]{17}(?:\\n|$)")))) {
							// An enrolling station is not an authenticated member yet.
							// Let its WPS/TLS attempt finish before merging empty groups.
							proposal_since = now;
							continue;
						}
						RCLCPP_INFO(logger_, "Fleet converging from %s to established owner %s",
							config_.name(owner).c_str(), config_.name(selected).c_str());
						if (discovered.count(selected)) {
							// Reformation clears the observation map. Preserve this
							// fresh owner's channel so retry starts there immediately.
							last_owner_frequency_ = discovered.at(selected).radio.frequency;
						}
						break;	// Cleanup only our group, then discover/join the winning group.
					}
					if (created.empty()) {
						owner = selected;
						if (!hosting && owner == config_.own && local.can_host && now >= settle_until) {
							hosting = true;
							local.role = FleetRole::hosting;
							local.owner = owner;
							publish_status();
							RCLCPP_INFO(logger_, "Fleet elected %s as Wi-Fi Direct owner", config_.name(owner).c_str());
							request.ok("P2P_STOP_FIND");
							// A failed join may have targeted another standalone channel.
							// Hosting always uses our own validated channel settings.
							frequency = std::to_string(local.radio.frequency);
							request.ok("P2P_GROUP_ADD freq=" + frequency + channel_options);
							host_requested = true;
							address_deadline = now + std::chrono::seconds(config_.address_timeout);
						} else if (owner && local.can_join && discovered.count(owner) &&
							now - discovered.at(owner).seen <= 3s && now >= next_join) {
							join_owner(discovered.at(owner));
						} else if (!hosting && owner && local.can_join && discovered.count(owner) && now >= next_scan) {
							discover();
							next_scan = now + 4s;
						}
					}
				}
				if (hosting && created.empty() && now >= address_deadline) {
					throw std::runtime_error("Elected owner failed to form a group");
				}
				// Empty groups continue discovery until a client arrives. An AP control
				// connection alone must not suppress the scans needed for a direct link.
				// Associated/enrolling stations and authenticated members keep their GO
				// scan-free. Every AP-associated scan stays on the primary channel.
				if (hosting && group_control && now >= next_isolated_scan) {
					const auto station = group_control->request("STA-FIRST");
					const bool clients = std::regex_search(station, std::regex("^[0-9A-Fa-f:]{17}(?:\\n|$)"));
					if (!clients && local.members == 1) {
						discover(true);
						refresh_peers();
					}
					next_isolated_scan = now + (ap_present ? 5s : 10s);
				}
				if (!hosting && created.empty() && !enrolling && now >= next_scan) {
					discover();
					next_scan = now +
						(!ap_present || (proposed && proposed != config_.own) ||
									now < settle_until + std::chrono::seconds(config_.peer_timeout)
								? 4s
								: 30s);
				}
			}
		} catch (...) {
			if (enrolling && owner && owner != config_.own) {
				failed_groups_[owner] = {
					joining_owner.ssid, Clock::now() + std::chrono::seconds(config_.host_cooldown)};
			}
			if (hosting && !addressed) {
				host_retry_ = Clock::now() + std::chrono::seconds(config_.host_cooldown);
				local.can_host = false;
			}
			cleanup();
			throw;
		}
		cleanup();
	}
	const Config& config_;
	rclcpp::Logger logger_;
	std::mutex mutex_;
	std::string group_, last_error_;
	DiscoveredOwners discovered_;
	NearbyUavs nearby_;
	FleetStatus fleet_status_;
	FleetPeers peers_;
	// Retry an unresponsive group after cooldown even if its SSID is unchanged.
	std::map<int, std::pair<std::string, Clock::time_point>> failed_groups_;
	RadioChannel cooldown_radio_;
	// Retain the reachable owner's channel across cleanup and retry backoff.
	uint32_t last_ap_frequency_{}, last_owner_frequency_{};
	Clock::time_point host_retry_{};
	std::atomic<bool> stop_{false};
	std::thread thread_;
	std::unique_ptr<RadioLease> lease_;
};

}  // namespace mrs_uav_wifi_direct
