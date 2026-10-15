// SPDX-License-Identifier: BSD-3-Clause
/** @file ros_bridge.hpp
 * @brief Create ROS subscriptions, received-topic publishers, and sending timers.
 */
#pragma once

#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/generic_subscription.hpp>

#include "quic_transport.hpp"
#include "wifi_direct.hpp"

namespace mrs_uav_wifi_direct {
/**
 * @brief Connect the configured ROS topics to the QUIC transport.
 * Subscriptions, send-rate timers and receive publishers run on one ROS executor.
 * MsQuic workers queue message buffers. The ROS executor creates and uses publishers.
 */
class WifiDirectNode : public rclcpp::Node {
	/** @brief Keep the source subscription, latest sample, and publishers for received data. */
	struct Topic {
		TopicConfig config;
		uint64_t sequence{};
		rclcpp::GenericSubscription::SharedPtr subscription;
		rclcpp::TimerBase::SharedPtr timer;
		std::shared_ptr<rclcpp::SerializedMessage> pending;
		std::map<int, rclcpp::GenericPublisher::SharedPtr> publishers;
		std::map<int, std::pair<uint64_t, uint64_t>> last_received;
	};

	public:
	/** @brief Load settings and message types, then start QUIC and Wi-Fi Direct. */
	WifiDirectNode() : Node("mrs_uav_wifi_direct") {
		config_ = std::make_unique<Config>(
			declare_parameter<std::string>("config_path", ""), declare_parameter<std::string>("hostname", ""));
		if (config_->numbers.empty()) {
			RCLCPP_INFO(get_logger(), "uav_list is empty; topic sharing, QUIC and Wi-Fi Direct are disabled");
			return;
		}
		config_->prefix = get_node_topics_interface()->resolve_topic_name(config_->prefix);
		wifi_ = std::make_unique<WifiDirect>(*config_, get_logger());
		transport_ = std::make_unique<QuicTransport>(*config_, get_logger());
		transport_->start();
		for (auto configured : config_->topics) {
			configured.source = get_node_topics_interface()->resolve_topic_name(configured.source);
			if (configured.source == config_->prefix ||
				configured.source.compare(0, config_->prefix.size() + 1, config_->prefix + "/") == 0) {
				throw std::runtime_error("Exported topic must be outside receive_prefix");
			}
			auto& topic = topics_[configured.key];
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
				configured.source, configured.type, qos,
				[this, key = configured.key](std::shared_ptr<rclcpp::SerializedMessage> message) {
					auto& entry = topics_.at(key);
					if (entry.config.hz > 0) {
						entry.pending = std::move(message);
					} else {
						transport_->send(entry.config, *message, ++entry.sequence);
					}
				},
				options);
			if (configured.hz > 0) {
				topic.timer = create_wall_timer(std::chrono::nanoseconds(static_cast<int64_t>(1e9 / configured.hz)),
					[this, key = configured.key]() {
						auto& entry = topics_.at(key);
						if (entry.pending) {
							transport_->send(entry.config, *entry.pending, ++entry.sequence);
							entry.pending.reset();
						}
					});
			}
			RCLCPP_INFO(get_logger(), "Sharing %s (%s), key=%s, rate_hz=%.3f", configured.source.c_str(),
				configured.type.c_str(), configured.key.c_str(), configured.hz);
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
	}

	private:
	/**
	 * @brief Publish configured peer payloads, rejecting duplicate/late topic sequences.
	 * Publishers are created on first receipt and their resolved names are checked
	 * against all outgoing subscriptions to prevent remapping-induced forwarding loops.
	 */
	void publish_received() {
		for (auto& message : transport_->drain()) {
			auto& topic = topics_.at(message.key);
			auto& last = topic.last_received[message.peer];
			if (last.first == message.instance && message.sequence <= last.second) {
				continue;
			}
			try {
				auto& publisher = topic.publishers[message.peer];
				if (!publisher) {
					const auto name = config_->prefix + "/" + config_->name(message.peer) + "/" + message.key;
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
				last = {message.instance, message.sequence};
			} catch (const std::exception& error) {
				RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Received publish failed: %s", error.what());
			}
		}
	}
	std::unique_ptr<Config> config_;
	std::unique_ptr<QuicTransport> transport_;
	std::unique_ptr<WifiDirect> wifi_;
	std::map<std::string, Topic> topics_;
	rclcpp::TimerBase::SharedPtr maintain_timer_, receive_timer_;
};

}  // namespace mrs_uav_wifi_direct
