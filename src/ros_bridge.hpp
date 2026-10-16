// SPDX-License-Identifier: BSD-3-Clause
/** @file ros_bridge.hpp
 * @brief Create ROS subscriptions, received-topic publishers, and sending timers.
 */
#pragma once

#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/generic_subscription.hpp>

#include "discovery_message.hpp"
#include "quic_transport.hpp"
#include "wifi_direct.hpp"

namespace mrs_uav_wifi_direct {
/**
 * @brief Connect the configured ROS topics to the QUIC transport.
 * Subscriptions, send-rate timers and receive publishers run on one ROS executor.
 * MsQuic workers queue message buffers. The ROS executor creates and uses publishers.
 */
class WifiDirectNode : public rclcpp::Node {
	/** @brief Received topic state stays on the ROS executor, including duplicate/rate tracking. */
	struct Import {
		rclcpp::GenericPublisher::SharedPtr publisher;
		uint64_t instance{}, sequence{}, messages{}, previous_messages{}, drops{};
		Clock::time_point last{};
	};
	/** @brief Keep the source subscription, latest sample, and publishers for received data. */
	struct Topic {
		TopicConfig config;
		uint64_t sequence{};
		rclcpp::GenericSubscription::SharedPtr subscription;
		rclcpp::TimerBase::SharedPtr timer;
		std::shared_ptr<rclcpp::SerializedMessage> pending;
		std::map<int, Import> imports;
		uint64_t source_messages{}, previous_source_messages{};
		Clock::time_point last_source{};
	};

	public:
	/** @brief Load settings and message types, then start QUIC and Wi-Fi Direct. */
	WifiDirectNode() : Node("mrs_uav_wifi_direct") {
		config_path_ = declare_parameter<std::string>("config_path", "");
		config_ = std::make_unique<Config>(config_path_, declare_parameter<std::string>("hostname", ""));
		if (config_->numbers.empty()) {
			RCLCPP_INFO(get_logger(), "uav_list is empty; collecting discovery data only");
			start_discovery();
			start_status();
			return;
		}
		// Resolve remappings before constructing the transport. Its stream paths
		// identify the actual subscribed topics, including the sending hostname.
		std::map<std::string, std::string> subscription_names;
		for (auto& configured : config_->topics) {
			const auto source = configured.source;
			configured.source = get_node_topics_interface()->resolve_topic_name(configured.source);
			configured.path = scoped_topic(configured.source, config_->hostname);
			if (configured.path.size() > 1024 || !subscription_names.emplace(configured.path, source).second) {
				throw std::runtime_error("Invalid or duplicate shared topic after ROS remapping: " + configured.path);
			}
		}
		for (const auto& configured : config_->topics) {
			// Check actual import names: a root prefix must still allow ordinary
			// exports, while forwarding an imported topic would create a loop.
			for (const auto peer : config_->numbers) {
				if (peer == config_->own) {
					continue;
				}
				for (const auto& received : config_->topics) {
					const auto imported = config_->peers_prefix + config_->peer_topic(received.path, peer);
					if (configured.source == get_node_topics_interface()->resolve_topic_name(imported)) {
						throw std::runtime_error("Exported topic overlaps a received peer topic: " + configured.source);
					}
				}
			}
		}
		wifi_ = std::make_unique<WifiDirect>(*config_, get_logger());
		transport_ = std::make_unique<QuicTransport>(*config_, get_logger());
		transport_->start();
		for (const auto& configured : config_->topics) {
			auto& topic = topics_[configured.path];
			topic.config = configured;
			rclcpp::SubscriptionOptions options;
			options.ignore_local_publications = true;
			// Best effort accepts sensor publishers of either reliability. For a
			// reliable source, requesting reliable local ROS delivery also prevents
			// large serialized samples from being lost before they reach QUIC.
			auto qos = rclcpp::QoS(10);
			if (configured.reliable) {
				qos.reliable();
			} else {
				qos.best_effort();
			}
			topic.subscription = create_generic_subscription(
				// Use the original name so ROS applies remapping exactly once.
				subscription_names.at(configured.path), configured.type, qos,
				[this, path = configured.path](std::shared_ptr<rclcpp::SerializedMessage> message) {
					auto& entry = topics_.at(path);
					++entry.source_messages;
					entry.last_source = Clock::now();
					if (entry.config.hz > 0) {
						entry.pending = std::move(message);
					} else {
						transport_->send(entry.config, *message, ++entry.sequence);
					}
				},
				options);
			if (configured.hz > 0) {
				topic.timer = create_wall_timer(std::chrono::nanoseconds(static_cast<int64_t>(1e9 / configured.hz)),
					[this, path = configured.path]() {
						auto& entry = topics_.at(path);
						if (entry.pending) {
							transport_->send(entry.config, *entry.pending, ++entry.sequence);
							entry.pending.reset();
						}
					});
			}
			RCLCPP_INFO(get_logger(), "Sharing %s (%s), exported_topic=%s, rate_hz=%.3f", configured.source.c_str(),
				configured.type.c_str(), configured.path.c_str(), configured.hz);
		}
		wifi_->start();
		transport_->tick(wifi_->ready_interface());
		maintain_timer_ = create_wall_timer(1s, [this]() {
			// Transfer snapshots without nesting Wi-Fi and MsQuic locks.
			wifi_->observe_peers(transport_->fleet_peers());
			transport_->tick(wifi_->ready_interface());
			transport_->send_fleet(wifi_->fleet_status());
		});
		receive_timer_ = create_wall_timer(5ms, [this]() { publish_received(); });
		RCLCPP_INFO(get_logger(), "%s ready; one QUIC stream per topic and peer", config_->name(config_->own).c_str());
		start_discovery();
		start_status();
	}

	private:
	/** @brief Always publish discovery, including when an empty fleet keeps QUIC and group management off. */
	void start_discovery() {
		discovery_publisher_ = create_publisher<msg::Discovery>(
			node_prefix(config_->hostname) + "/discovery", rclcpp::QoS(1).reliable().transient_local());
		discovery_ = std::make_unique<RadioDiscovery>(*config_);
		publish_discovery();
		discovery_timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
												 std::chrono::duration<double>(config_->discovery_interval)),
			[this]() { publish_discovery(); });
	}
	/**
	 * @brief Publish current snapshots and independently measured per-topic rates.
	 * Logging and discovery have separate counter baselines so either interval can
	 * change without corrupting the other's rates. Counters stay on the ROS executor.
	 */
	void publish_discovery() {
		const auto now = Clock::now();
		const auto transport = transport_ ? transport_->status() : TransportStatus{};
		const auto wifi = wifi_ ? wifi_->status() : WifiStatus{};
		auto output = discovery_message(*config_, discovery_->status(), wifi, transport, now);
		output.header.stamp = get_clock()->now();
		const double elapsed =
			last_discovery_ == Clock::time_point{} ? 0 : std::chrono::duration<double>(now - last_discovery_).count();
		auto rate = [elapsed](uint64_t count, uint64_t previous) {
			return elapsed > 0 ? (count - previous) / elapsed : std::numeric_limits<double>::quiet_NaN();
		};
		for (const auto peer : config_->numbers) {
			if (peer == config_->own) {
				continue;
			}
			std::vector<msg::TopicStatus> shared;
			for (const auto& [path, topic] : topics_) {
				const auto id = std::make_pair(peer, path);
				const auto sent = transport.exported.find(id);
				const auto traffic = sent == transport.exported.end() ? TopicTraffic{} : sent->second;
				const auto found = topic.imports.find(peer);
				const auto imported = found == topic.imports.end() ? Import{} : found->second;
				const auto dropped = transport.receive_drops_by_topic.find(id);
				auto& previous = previous_discovery_[id];
				msg::TopicStatus item;
				item.exported_topic = path;
				item.source_topic = topic.config.source;
				item.received_topic = config_->peers_prefix + config_->peer_topic(path, peer);
				item.message_type = topic.config.type;
				item.rate_limit_hz = topic.config.hz;
				item.source_reliable = topic.config.reliable;
				item.source_present = topic.subscription->get_publisher_count() != 0;
				item.source_hz = rate(topic.source_messages, previous[0]);
				item.exported_hz = rate(traffic.messages, previous[1]);
				item.imported_hz = rate(imported.messages, previous[2]);
				previous = {topic.source_messages, traffic.messages, imported.messages};
				item.exported_messages = traffic.messages;
				item.imported_messages = imported.messages;
				item.export_drops = traffic.drops;
				item.import_drops =
					imported.drops + (dropped == transport.receive_drops_by_topic.end() ? 0 : dropped->second);
				item.source_age_s = observation_age(topic.last_source, now);
				item.import_age_s = observation_age(imported.last, now);
				shared.push_back(std::move(item));
			}
			for (auto& entry : output.uavs) {
				if (entry.hostname == config_->name(peer)) {
					entry.topics = shared;
				}
			}
		}
		discovery_publisher_->publish(output);
		last_discovery_ = now;
	}
	/** @brief Use a wall timer so reports continue when the ROS simulation clock is paused. */
	void start_status() {
		if (config_->status_interval == 0) {
			return;
		}
		print_status();
		status_timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
											  std::chrono::duration<double>(config_->status_interval)),
			[this]() { print_status(); });
	}
	/**
	 * @brief Report configuration and measured topic rates from independent state snapshots.
	 * Export rates count samples accepted by each QUIC stream, before acknowledgment.
	 * Import rates count successful ROS publishes after duplicate filtering. Every rate
	 * uses the actual steady-clock time since the previous report. No radio queries or
	 * network operations run in this callback; the first report has no rate history.
	 */
	void print_status() {
		const auto transport = transport_ ? transport_->status() : TransportStatus{};
		auto wifi = wifi_ ? wifi_->status() : WifiStatus{};
		const auto radio = discovery_->status();
		wifi.nearby = nearby_observations(radio, wifi, Clock::now(), config_->peer_timeout);
		const auto now = Clock::now();
		const double elapsed =
			last_status_ == Clock::time_point{} ? 0 : std::chrono::duration<double>(now - last_status_).count();
		auto rate = [elapsed](uint64_t count, uint64_t& previous) {
			const double result = elapsed > 0 ? static_cast<double>(count - previous) / elapsed : 0;
			previous = count;
			return result;
		};
		const auto drops = transport.send_drops + transport.receive_drops;
		std::ostringstream out;
		bool missing_source = false;
		std::vector<std::string> definitions;
		std::map<int, std::vector<std::string>> exports, imports;
		for (auto& [path, topic] : topics_) {
			const auto source_rate = rate(topic.source_messages, topic.previous_source_messages);
			const auto publishers = topic.subscription->get_publisher_count();
			missing_source |= publishers == 0;
			const char* source_health = !publishers ? "NO_PUBLISHER"
				: !topic.source_messages			? "WAITING_DATA"
				: source_rate > 0					? "ACTIVE"
													: "IDLE";
			definitions.push_back(path + "(source=" + topic.subscription->get_topic_name() + "," +
				"limit=" + (topic.config.hz > 0 ? status_number(topic.config.hz) + "Hz" : "unlimited") +
				",rate=" + status_number(source_rate) + "Hz)");
			for (const auto peer : config_->numbers) {
				if (peer == config_->own) {
					continue;
				}
				const auto id = std::make_pair(peer, path);
				const auto sent = transport.exported.find(id);
				const auto traffic = sent == transport.exported.end() ? TopicTraffic{} : sent->second;
				std::string exported = path + "=" + status_number(rate(traffic.messages, previous_exported_[id])) +
					"Hz,drops=" + std::to_string(traffic.drops) + "," + (topic.config.reliable ? "R" : "BE");
				if (source_rate == 0 || publishers == 0) {
					exported += "," + std::string(source_health) + "/" + status_age(topic.last_source, now);
				}
				exports[peer].push_back(std::move(exported));
				auto& imported = topic.imports[peer];
				const auto received_rate = rate(imported.messages, imported.previous_messages);
				const auto dropped = transport.receive_drops_by_topic.find(id);
				const auto receive_drops =
					imported.drops + (dropped == transport.receive_drops_by_topic.end() ? 0 : dropped->second);
				std::string received = config_->peer_topic(path, peer) + "=" + status_number(received_rate) +
					"Hz,drops=" + std::to_string(receive_drops) + ",R";
				if (received_rate == 0) {
					received +=
						std::string(imported.messages ? ",IDLE/" : ",WAITING_DATA/") + status_age(imported.last, now);
				}
				imports[peer].push_back(std::move(received));
			}
		}
		if (!config_->numbers.empty()) {
			out << "\n  topics=" << status_list(definitions) << " receive=" << config_->peers_prefix
				<< "/<peer>/<topic path>"
				<< " | ROS QoS: R=reliable BE=best_effort; QUIC=reliable";
		}
		for (const auto peer : config_->numbers) {
			if (peer == config_->own) {
				continue;
			}
			const auto entry = transport.connected.find(peer);
			const auto paths = entry == transport.connected.end() ? PeerConnections{} : entry->second;
			const auto report = transport.peers.find(peer);
			const auto route = status_route(*config_, paths);
			const char* health = entry == transport.connected.end() ? "DOWN"
				: std::string(route) == "no_route"					? "NO_ROUTE"
				: report == transport.peers.end()					? "WAITING_STATUS"
				: config_->wifi && !paths.direct					? "AP_FALLBACK"
																	: "OK";
			std::vector<std::string> latency;
			if (paths.ap) {
				latency.push_back("AP=" + status_latency(paths.ap_latency, now, config_->peer_timeout));
			}
			if (paths.direct) {
				latency.push_back("direct=" + status_latency(paths.direct_latency, now, config_->peer_timeout));
			}
			out << "\n  peer " << config_->name(peer) << " link=" << health << " paths="
				<< (paths.ap			  ? "AP"
						   : paths.direct ? ""
										  : "none")
				<< (paths.ap && paths.direct ? "+" : "") << (paths.direct ? "direct" : "") << " send=" << route
				<< " heartbeat=" << (report == transport.peers.end() ? "missing" : status_age(report->second.seen, now))
				<< " RTT=" << status_list(latency) << " out=" << status_list(exports[peer])
				<< " in=" << status_list(imports[peer]);
		}
		const auto summary = status_summary(*config_, config_path_, wifi, transport, now, drops - previous_drops_,
			publish_errors_ - previous_publish_errors_, missing_source);
		out << "\n  radio=" << (radio.available ? "available" : "unavailable")
			<< " driver=" << (radio.driver.empty() ? "unknown" : radio.driver)
			<< " age=" << status_age(radio.sampled, now)
			<< " AP=" << (radio.ap.connected ? radio.ap.ssid : "disconnected");
		if (std::isfinite(radio.ap.signal_dbm)) {
			out << " signal=" << radio.ap.signal_dbm << "dBm";
		}
		if (!radio.error.empty()) {
			out << " radio_error=" << radio.error;
		}
		if (!radio.discovery_error.empty()) {
			out << " discovery_error=" << radio.discovery_error;
		}
		RCLCPP_INFO(get_logger(), "%s%s", summary.c_str(), out.str().c_str());
		last_status_ = now;
		previous_drops_ = drops;
		previous_publish_errors_ = publish_errors_;
	}
	/**
	 * @brief Publish configured peer payloads, rejecting duplicate/late topic sequences.
	 * Publishers are created on first receipt and their resolved names are checked
	 * against all outgoing subscriptions to prevent remapping-induced forwarding loops.
	 */
	void publish_received() {
		for (auto& message : transport_->drain()) {
			auto& topic = topics_.at(message.path);
			auto& imported = topic.imports[message.peer];
			if (imported.instance == message.instance && message.sequence <= imported.sequence) {
				continue;
			}
			try {
				auto& publisher = imported.publisher;
				if (!publisher) {
					const auto name = config_->peers_prefix + config_->peer_topic(message.path, message.peer);
					const auto resolved = get_node_topics_interface()->resolve_topic_name(name);
					for (const auto& entry : topics_) {
						if (resolved == entry.second.subscription->get_topic_name()) {
							throw std::runtime_error("Received topic remaps to an exported topic: " + resolved);
						}
					}
					publisher = create_generic_publisher(name, topic.config.type, rclcpp::QoS(10));
					RCLCPP_INFO(get_logger(), "Receiving %s", publisher->get_topic_name());
				}
				rclcpp::SerializedMessage output(message.payload.size());
				auto& serialized = output.get_rcl_serialized_message();
				std::memcpy(serialized.buffer, message.payload.data(), message.payload.size());
				serialized.buffer_length = message.payload.size();
				publisher->publish(output);
				imported.instance = message.instance;
				imported.sequence = message.sequence;
				++imported.messages;
				imported.last = Clock::now();
			} catch (const std::exception& error) {
				++publish_errors_;
				++imported.drops;
				RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Received publish failed: %s", error.what());
			}
		}
	}
	std::string config_path_;
	std::unique_ptr<Config> config_;
	std::unique_ptr<QuicTransport> transport_;
	std::unique_ptr<WifiDirect> wifi_;
	std::unique_ptr<RadioDiscovery> discovery_;
	rclcpp::Publisher<msg::Discovery>::SharedPtr discovery_publisher_;
	std::map<std::string, Topic> topics_;
	rclcpp::TimerBase::SharedPtr maintain_timer_, receive_timer_, status_timer_, discovery_timer_;
	Clock::time_point last_status_{}, last_discovery_{};
	std::map<std::pair<int, std::string>, std::array<uint64_t, 3>> previous_discovery_;
	std::map<std::pair<int, std::string>, uint64_t> previous_exported_;
	uint64_t previous_drops_{}, publish_errors_{}, previous_publish_errors_{};
};

}  // namespace mrs_uav_wifi_direct
