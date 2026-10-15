// SPDX-License-Identifier: BSD-3-Clause
/** @file quic_transport.hpp
 * @brief QUIC connections that verify both UAV identities, topic streams, and queue limits.
 */
#pragma once

#include <msquic.h>
#include <net/if.h>
#include <openssl/x509.h>
#include <openssl/x509v3.h>

#include <cstring>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <random>
#include <rclcpp/rclcpp.hpp>

#include "certificates.hpp"
#include "fleet_policy.hpp"
#include "network_utils.hpp"
#include "status.hpp"

namespace mrs_uav_wifi_direct {
/**
 * @brief Encrypted per-peer connections with one reliable stream per topic/direction.
 * MsQuic invokes callbacks on its own workers; mutable handles, queues and stream
 * buffers are guarded by mutex_. ROS calls tick/send/drain from one executor.
 * Blocking close/SetParam operations must run outside that mutex so a waiting
 * worker can complete callbacks. Callback contexts stay alive until ConnectionClose
 * has synchronized all callbacks, then their owning connection is released.
 */
class QuicTransport {
	struct Connection;
	/** @brief Stable callback context and partial-frame state for a single topic channel. */
	struct Stream {
		Connection* connection{};
		HQUIC handle{};
		std::string key;
		uint64_t instance{};
		size_t pending{};
		bool header{}, fleet{};
		uint64_t sequence{};
		std::vector<uint8_t> input;
	};
	/** @brief Own every stream context until this connection has been synchronously closed. */
	struct Connection {
		QuicTransport* transport{};
		HQUIC handle{};
		int peer{};
		bool direct{}, connected{}, finished{};
		Stream* fleet_outgoing{};
		Stream* fleet_incoming{};
		FleetObservation observation;
		PathLatency latency;
		std::map<std::string, Stream*> outgoing;
		std::vector<std::unique_ptr<Stream>> streams;
	};
	/** @brief Address-filtered AP/direct listener with a stable callback context. */
	struct Listener {
		QuicTransport* transport{};
		bool direct{};
		HQUIC handle{};
	};
	/** @brief Keep both QUIC_BUFFER descriptor and payload alive until SEND_COMPLETE. */
	struct SendBuffer {
		std::vector<uint8_t> data;
		QUIC_BUFFER buffer{};
	};

	public:
	/** @brief Allocate an instance ID used by every topic stream for this process lifetime. */
	QuicTransport(const Config& config, rclcpp::Logger logger)
		: config_(config),
		  logger_(logger),
		  certificates_(config, logger),
		  ap_listener_{this, false, nullptr},
		  direct_listener_{this, true, nullptr} {
		std::random_device random;
		instance_ = (static_cast<uint64_t>(random()) << 32) | random();
		// A fleet remains the same when a UAV loses its AP or changes local subnets.
		std::vector<std::string> fields{std::to_string(config_.port), std::to_string(config_.port_per_uav)};
		for (const auto& [number, name] : config_.names) {
			fields.push_back(name);
		}
		fleet_hash_ = fleet_hash(fields);
	}
	/** @brief Synchronize transport callbacks before releasing contexts and credentials. */
	~QuicTransport() {
		stop();
	}
	/** @brief Load certificates and configure the topic streams. */
	void start() {
		certificates_.prepare();
		check(MsQuicOpen2(&api_), "MsQuicOpen2");
		const QUIC_REGISTRATION_CONFIG registration{"mrs_uav_wifi_direct", QUIC_EXECUTION_PROFILE_LOW_LATENCY};
		check(api_->RegistrationOpen(&registration, &registration_), "RegistrationOpen");
		QUIC_SETTINGS settings{};
		settings.IsSet.PeerUnidiStreamCount = TRUE;
		settings.PeerUnidiStreamCount = 1025;  // Topic channels plus one independent fleet stream.
		settings.IsSet.KeepAliveIntervalMs = TRUE;
		settings.KeepAliveIntervalMs = 2000;
		settings.IsSet.IdleTimeoutMs = TRUE;
		settings.IdleTimeoutMs = 15000;
		settings.IsSet.DisconnectTimeoutMs = TRUE;
		settings.DisconnectTimeoutMs = 3000;
		settings.IsSet.HandshakeIdleTimeoutMs = TRUE;
		settings.HandshakeIdleTimeoutMs = 3000;
		// Keep send buffers until acknowledged so pending-byte limits bound network backlog.
		settings.IsSet.SendBufferingEnabled = TRUE;
		settings.SendBufferingEnabled = FALSE;
		settings.IsSet.MigrationEnabled = TRUE;
		settings.MigrationEnabled = FALSE;	// Separate pinned AP/direct paths, explicit switchover.
		for (const bool client : {false, true}) {
			HQUIC& handle = client ? client_config_ : server_config_;
			auto alpn = protocol();
			check(api_->ConfigurationOpen(registration_, &alpn, 1, &settings, sizeof(settings), nullptr, &handle),
				"ConfigurationOpen");
			QUIC_CERTIFICATE_FILE certificate{config_.key.c_str(), config_.cert.c_str()};
			QUIC_CREDENTIAL_CONFIG credential{};
			credential.Type = QUIC_CREDENTIAL_TYPE_CERTIFICATE_FILE;
			credential.CertificateFile = &certificate;
			credential.CaCertificateFile = config_.ca.empty() ? nullptr : config_.ca.c_str();
			// In automatic mode our callback performs signature, identity and pin checks;
			// the platform CA store cannot validate independently generated leaf identities.
			credential.Flags = static_cast<QUIC_CREDENTIAL_FLAGS>(
				(config_.ca.empty() ? QUIC_CREDENTIAL_FLAG_NO_CERTIFICATE_VALIDATION
									: QUIC_CREDENTIAL_FLAG_SET_CA_CERTIFICATE_FILE) |
				QUIC_CREDENTIAL_FLAG_INDICATE_CERTIFICATE_RECEIVED | QUIC_CREDENTIAL_FLAG_USE_PORTABLE_CERTIFICATES |
				(client ? QUIC_CREDENTIAL_FLAG_CLIENT : QUIC_CREDENTIAL_FLAG_REQUIRE_CLIENT_AUTHENTICATION));
			check(api_->ConfigurationLoadCredential(handle, &credential), "ConfigurationLoadCredential");
		}
	}

	/**
	 * @brief Update listeners and peer connections as host interfaces change.
	 * The lower-numbered UAV starts each connection, which avoids duplicate connections.
	 * Both AP and direct connections may remain established; send selects one path.
	 */
	void tick(const std::string& direct_interface) {
		auto ap_network = config_.ap && interface_carrier(config_.interface) ? interface_network(config_.interface)
																			 : InterfaceNetwork{};
		try {
			if (ap_network) {
				for (const auto peer : config_.numbers) {
					ap_network.peer_address(config_.own, peer);
				}
			}
		} catch (const std::exception& error) {
			warn(error.what());
			ap_network = {};
		}
		auto direct_network =
			interface_carrier(direct_interface) ? interface_network(direct_interface) : InterfaceNetwork{};
		try {
			if (direct_network) {
				for (const auto peer : config_.numbers) {
					direct_network.peer_address(config_.own, peer);
				}
				const auto infrastructure = interface_network(config_.interface);
				const auto common_mask = direct_network.mask & infrastructure.mask;
				if (infrastructure && (direct_network.ip & common_mask) == (infrastructure.ip & common_mask)) {
					throw std::runtime_error("P2P subnet overlaps the managed interface's subnet");
				}
			}
		} catch (const std::exception& error) {
			warn(error.what());
			direct_network = {};
		}
		// Closing handles is synchronous; never hold the callback mutex while closing.
		if (!(direct_network == direct_network_)) {
			if (direct_listener_.handle) {
				api_->ListenerStop(direct_listener_.handle);
				api_->ListenerClose(direct_listener_.handle);
				direct_listener_.handle = nullptr;
			}
			std::lock_guard<std::recursive_mutex> lock(mutex_);
			for (const auto& connection : connections_) {
				if (connection->direct && !connection->finished) {
					connection->connected = false;
					api_->ConnectionShutdown(connection->handle, QUIC_CONNECTION_SHUTDOWN_FLAG_NONE, 0);
				}
			}
			direct_network_ = direct_network;
			direct_interface_ = direct_network ? direct_network.interface : "";
			next_listener_ = Clock::time_point{};
		}
		if (!(ap_network == ap_network_)) {
			if (ap_listener_.handle) {
				api_->ListenerStop(ap_listener_.handle);
				api_->ListenerClose(ap_listener_.handle);
				ap_listener_.handle = nullptr;
			}
			std::lock_guard<std::recursive_mutex> lock(mutex_);
			for (const auto& connection : connections_) {
				if (!connection->direct && !connection->finished) {
					connection->connected = false;
					api_->ConnectionShutdown(connection->handle, QUIC_CONNECTION_SHUTDOWN_FLAG_NONE, 0);
				}
			}
			ap_network_ = ap_network;
			next_listener_ = Clock::time_point{};
		}
		if (Clock::now() >= next_listener_) {
			next_listener_ = Clock::now() + 5s;
			if (ap_network_ && !ap_listener_.handle) {
				listen(ap_listener_);
			}
			if (!direct_interface_.empty() && !direct_listener_.handle) {
				listen(direct_listener_);
			}
		}
		std::vector<std::unique_ptr<Connection>> retired;
		std::vector<std::pair<int, bool>> dial;
		{
			std::lock_guard<std::recursive_mutex> lock(mutex_);
			for (auto it = connections_.begin(); it != connections_.end();) {
				if ((*it)->finished) {
					retired.push_back(std::move(*it));
					it = connections_.erase(it);
				} else {
					++it;
				}
			}
			for (const auto peer : config_.numbers) {
				if (peer <= config_.own) {
					continue;
				}  // Exactly one initiator for each pair.
				for (const bool direct : {false, true}) {
					if ((direct && (!direct_listener_.handle || direct_interface_.empty())) ||
						(!direct && !ap_listener_.handle)) {
						continue;
					}
					const auto id = std::make_pair(peer, direct);
					const bool exists = std::any_of(connections_.begin(), connections_.end(),
						[&](const auto& c) { return c->peer == peer && c->direct == direct && !c->finished; });
					if (!exists && Clock::now() >= next_connect_[id]) {
						next_connect_[id] = Clock::now() + 5s;
						dial.emplace_back(peer, direct);
					}
				}
			}
		}
		for (const auto& connection : retired) {
			api_->ConnectionClose(connection->handle);
		}
		// SetParam may synchronously wait for a MsQuic worker. A callback on that
		// worker can need mutex_, so connection creation must run outside the lock.
		for (const auto& endpoint : dial) {
			connect(endpoint.first, endpoint.second);
		}
	}

	/**
	 * @brief Send a serialized ROS sample to each connected UAV.
	 * A disconnected UAV receives new samples after reconnecting. Each topic has a
	 * send-buffer limit; a full buffer drops new samples on that topic.
	 */
	void send(const TopicConfig& topic, const rclcpp::SerializedMessage& message, uint64_t sequence) {
		const auto& data = message.get_rcl_serialized_message();
		std::lock_guard<std::recursive_mutex> lock(mutex_);
		if (data.buffer_length > config_.max_message) {
			++send_drops_;
			for (const auto peer : config_.numbers) {
				if (peer != config_.own) {
					++exported_[{peer, topic.key}].drops;
				}
			}
			warn("Dropping oversized ROS message on " + topic.key);
			return;
		}
		for (const auto peer : config_.numbers) {
			if (peer == config_.own) {
				continue;
			}
			Connection* chosen{};
			for (const auto& candidate : connections_) {
				if (candidate->peer != peer || !candidate->connected || candidate->finished) {
					continue;
				}
				if (candidate->direct) {
					chosen = candidate.get();
					break;
				}
				if (config_.fallback && config_.ap) {
					chosen = candidate.get();
				}
			}
			if (!chosen) {
				continue;
			}
			auto& traffic = exported_[{peer, topic.key}];
			try {
				auto& stream = chosen->outgoing[topic.key];
				if (!stream || !stream->handle) {
					stream = open_stream(*chosen, topic);
				}
				std::vector<uint8_t> frame;
				frame.reserve(12 + data.buffer_length);
				append(frame, sequence, 8);
				append(frame, data.buffer_length, 4);
				frame.insert(frame.end(), data.buffer, data.buffer + data.buffer_length);
				if (queue(*stream, std::move(frame))) {
					++traffic.messages;
				} else {
					++traffic.drops;
				}
			} catch (const std::exception& error) {
				++traffic.drops;
				++send_drops_;
				warn(error.what());
			}
		}
	}

	/**
	 * @brief Send one small status update on every authenticated AP/direct connection.
	 * At most one unacknowledged update is queued per path, preventing an old backlog
	 * from keeping a dead owner alive. It works with no shared_topics configured.
	 * AP traffic is never created when ap_enabled is false.
	 */
	void send_fleet(const FleetStatus& status) {
		std::lock_guard<std::recursive_mutex> lock(mutex_);
		const auto sequence = ++fleet_sequence_;
		for (const auto& connection : connections_) {
			if (!connection->connected || connection->finished) {
				continue;
			}
			try {
				auto*& stream = connection->fleet_outgoing;
				if (!stream || !stream->handle) {
					stream = open_fleet_stream(*connection);
				}
				if (stream->pending == 0) {
					queue(*stream, fleet_record(status, sequence, fleet_hash_));
				}
			} catch (const std::exception& error) {
				warn(error.what());
			}
		}
	}
	/** @brief Copy fresh reports from live authenticated paths, deduplicating AP/direct. */
	FleetPeers fleet_peers() {
		std::lock_guard<std::recursive_mutex> lock(mutex_);
		FleetPeers result;
		const auto now = Clock::now();
		for (const auto& connection : connections_) {
			const auto& report = connection->observation;
			if (!connection->connected || connection->finished || !report.sequence ||
				now - report.seen > std::chrono::seconds(config_.peer_timeout)) {
				continue;
			}
			auto& current = result[connection->peer];
			if (!current.sequence || (current.instance == report.instance && current.sequence < report.sequence) ||
				(current.instance != report.instance && current.seen < report.seen)) {
				current = report;
			}
		}
		return result;
	}

	/** @brief Snapshot live paths and queue counters without calling MsQuic while logging. */
	TransportStatus status() {
		std::lock_guard<std::recursive_mutex> lock(mutex_);
		TransportStatus result;
		result.ap = ap_network_;
		result.direct = direct_network_;
		result.ap_listening = ap_listener_.handle != nullptr;
		result.direct_listening = direct_listener_.handle != nullptr;
		result.peers = fleet_peers();
		result.exported = exported_;
		result.receive_drops_by_topic = receive_drops_by_topic_;
		result.receive_bytes = receive_bytes_;
		result.receive_messages = received_.size();
		result.send_drops = send_drops_;
		result.receive_drops = receive_drops_;
		for (const auto& connection : connections_) {
			if (connection->finished) {
				continue;
			}
			if (connection->connected) {
				auto& peer = result.connected[connection->peer];
				(connection->direct ? peer.direct : peer.ap) = true;
				(connection->direct ? peer.direct_latency : peer.ap_latency) = connection->latency;
			} else {
				++result.pending_connections;
			}
			for (const auto& stream : connection->streams) {
				result.send_bytes += stream->pending;
				result.streams += stream->handle != nullptr;
			}
		}
		return result;
	}

	/** @brief Transfer ownership of queued receive payloads to the ROS executor. */
	std::deque<Received> drain() {
		std::lock_guard<std::recursive_mutex> lock(mutex_);
		std::deque<Received> messages;
		messages.swap(received_);
		receive_bytes_ = 0;
		return messages;
	}

	private:
	/**
	 * @brief Read RTT estimates on the connection's MsQuic worker after a heartbeat.
	 * GetParam executes inline on that worker. Calling it there avoids a blocking
	 * cross-thread query under mutex_; status logging only copies the stored result.
	 * The estimates come from existing QUIC acknowledgments and create no probes.
	 */
	void update_latency(Connection& connection) {
		QUIC_STATISTICS_V2 statistics{};
		uint32_t length = sizeof(statistics);
		if (QUIC_SUCCEEDED(api_->GetParam(connection.handle, QUIC_PARAM_CONN_STATISTICS_V2, &length, &statistics)) &&
			statistics.Rtt && statistics.MinRtt != UINT32_MAX && statistics.MaxRtt) {
			connection.latency = {
				statistics.Rtt, statistics.MinRtt, statistics.MaxRtt, statistics.RttVariance, Clock::now()};
		}
	}
	/** @brief Version the application protocol independently of QUIC/TLS. */
	static QUIC_BUFFER protocol() {
		static uint8_t name[] = "mrs-quic-ros2";
		return QUIC_BUFFER{sizeof(name) - 1, name};
	}
	/** @brief Report a failed MsQuic operation with its status code. */
	static void check(QUIC_STATUS status, const char* operation) {
		if (QUIC_FAILED(status)) {
			throw std::runtime_error(std::string(operation) + " failed (" + std::to_string(status) + ")");
		}
	}
	/** @brief Throttle worker-thread warnings using monotonic time. */
	void warn(const std::string& message) {
		std::lock_guard<std::recursive_mutex> lock(mutex_);
		if (Clock::now() >= next_warning_) {
			RCLCPP_WARN(logger_, "%s", message.c_str());
			next_warning_ = Clock::now() + 5s;
		}
	}
	/** @brief Create a MsQuic address from a UAV's assigned IPv4 address and port. */
	QUIC_ADDR address(int number, bool direct, uint16_t port) {
		QUIC_ADDR result{};
		const auto ip = (direct ? direct_network_ : ap_network_).peer_address(config_.own, number);
		if (!QuicAddrFromString(ip.c_str(), port, &result)) {
			throw std::runtime_error("Invalid IPv4 address");
		}
		return result;
	}
	/** @brief Start an address-filtered listener; retry errors on a later maintenance tick. */
	void listen(Listener& listener) {
		try {
			check(api_->ListenerOpen(registration_, listener_callback, &listener, &listener.handle), "ListenerOpen");
			auto alpn = protocol();
			const auto local = address(config_.own, listener.direct, config_.listen_port(config_.own));
			check(api_->ListenerStart(listener.handle, &alpn, 1, &local), "ListenerStart");
			RCLCPP_INFO(logger_, "QUIC %s listener on %s:%d", listener.direct ? "direct" : "AP",
				(listener.direct ? direct_network_.address : ap_network_.address).c_str(),
				config_.listen_port(config_.own));
		} catch (const std::exception& error) {
			if (listener.handle) {
				api_->ListenerClose(listener.handle);
				listener.handle = nullptr;
			}
			warn(error.what());
		}
	}
	/**
	 * @brief Connect to the peer's IPv4 address and check its certificate against the UAV name.
	 * Direct connections pin both the source IP and interface index. SetParam can
	 * wait for a worker, so this function is called without the callback mutex held.
	 */
	void connect(int peer, bool direct) {
		auto connection = std::make_unique<Connection>();
		connection->transport = this;
		connection->peer = peer;
		connection->direct = direct;
		auto* state = connection.get();
		try {
			check(api_->ConnectionOpen(registration_, connection_callback, state, &state->handle), "ConnectionOpen");
			const auto local = address(config_.own, direct, 0);
			const auto remote = address(peer, direct, config_.listen_port(peer));
			check(api_->SetParam(state->handle, QUIC_PARAM_CONN_LOCAL_ADDRESS, sizeof(local), &local), "Local address");
			check(api_->SetParam(state->handle, QUIC_PARAM_CONN_REMOTE_ADDRESS, sizeof(remote), &remote),
				"Remote address");
			if (direct) {
				const uint32_t index = ::if_nametoindex(direct_interface_.c_str());
				if (!index) {
					throw std::runtime_error("Direct interface disappeared");
				}
				check(api_->SetParam(state->handle, QUIC_PARAM_CONN_LOCAL_INTERFACE, sizeof(index), &index),
					"Direct interface");
				const auto dscp = static_cast<uint8_t>(config_.direct_dscp);
				// CS1/background avoids competing with ordinary AP packets in WMM.
				// MsQuic marks each connection's packets; AP-path priority stays unchanged.
				check(api_->SetParam(state->handle, QUIC_PARAM_CONN_SEND_DSCP, sizeof(dscp), &dscp), "Direct DSCP");
			}
			const QUIC_STREAM_SCHEDULING_SCHEME scheduling = QUIC_STREAM_SCHEDULING_SCHEME_ROUND_ROBIN;
			check(api_->SetParam(
					  state->handle, QUIC_PARAM_CONN_STREAM_SCHEDULING_SCHEME, sizeof(scheduling), &scheduling),
				"Stream scheduling");
			// Numeric remote address is set explicitly; SNI identifies the peer's certificate.
			{
				std::lock_guard<std::recursive_mutex> lock(mutex_);
				connections_.push_back(std::move(connection));
			}
			check(api_->ConnectionStart(state->handle, client_config_, QUIC_ADDRESS_FAMILY_INET,
					  config_.name(peer).c_str(), config_.listen_port(peer)),
				"ConnectionStart");
		} catch (const std::exception& error) {
			// The next maintenance tick closes failed connections outside the callback lock.
			std::lock_guard<std::recursive_mutex> lock(mutex_);
			state->finished = true;
			if (state->handle && connection) {
				connections_.push_back(std::move(connection));
			}
			warn(error.what());
		}
	}
	/** @brief Create a topic channel and queue its versioned sender/key/type header once. */
	Stream* open_stream(Connection& connection, const TopicConfig& topic) {
		auto stream = std::make_unique<Stream>();
		stream->connection = &connection;
		stream->key = topic.key;
		auto* state = stream.get();
		check(api_->StreamOpen(
				  connection.handle, QUIC_STREAM_OPEN_FLAG_UNIDIRECTIONAL, stream_callback, state, &state->handle),
			"StreamOpen");
		connection.streams.push_back(std::move(stream));
		const auto status = api_->StreamStart(state->handle, QUIC_STREAM_START_FLAG_IMMEDIATE);
		if (QUIC_FAILED(status)) {
			api_->StreamClose(state->handle);
			state->handle = nullptr;
			check(status, "StreamStart");
		}
		std::vector<uint8_t> header{'Q', 'C', 'R', '1'};
		append(header, config_.own, 2);
		append(header, instance_, 8);
		append(header, topic.key.size(), 2);
		append(header, topic.type.size(), 2);
		header.insert(header.end(), topic.key.begin(), topic.key.end());
		header.insert(header.end(), topic.type.begin(), topic.type.end());
		queue(*state, std::move(header));
		RCLCPP_INFO(logger_, "Topic channel %s -> %s over %s", topic.key.c_str(), config_.name(connection.peer).c_str(),
			connection.direct ? "Wi-Fi Direct" : "AP");
		return state;
	}
	/** @brief Open the single fleet control stream with a distinct 14-byte versioned header. */
	Stream* open_fleet_stream(Connection& connection) {
		auto stream = std::make_unique<Stream>();
		stream->connection = &connection;
		stream->fleet = true;
		auto* state = stream.get();
		check(api_->StreamOpen(
				  connection.handle, QUIC_STREAM_OPEN_FLAG_UNIDIRECTIONAL, stream_callback, state, &state->handle),
			"Fleet StreamOpen");
		connection.streams.push_back(std::move(stream));
		const auto status = api_->StreamStart(state->handle, QUIC_STREAM_START_FLAG_IMMEDIATE);
		if (QUIC_FAILED(status)) {
			api_->StreamClose(state->handle);
			state->handle = nullptr;
			check(status, "Fleet StreamStart");
		}
		std::vector<uint8_t> header{'Q', 'C', 'F', '2'};
		append(header, config_.own, 2);
		append(header, instance_, 8);
		queue(*state, std::move(header));
		return state;
	}
	/** @brief Read fleet reports and reject duplicate streams or repeated sequence numbers. */
	bool parse_fleet(Stream& stream) {
		auto& bytes = stream.input;
		if (!stream.header) {
			if (bytes.size() < 14) {
				return true;
			}
			if (integer(bytes.data() + 4, 2) != static_cast<uint64_t>(stream.connection->peer) ||
				(stream.connection->fleet_incoming && stream.connection->fleet_incoming->handle &&
					stream.connection->fleet_incoming != &stream)) {
				return false;
			}
			stream.connection->fleet_incoming = &stream;
			stream.instance = integer(bytes.data() + 6, 8);
			stream.header = true;
			bytes.erase(bytes.begin(), bytes.begin() + 14);
		}
		size_t consumed = 0;
		while (bytes.size() - consumed >= 36) {
			const auto* record = bytes.data() + consumed;
			const auto sequence = integer(record, 8);
			FleetStatus status;
			if (sequence <= stream.sequence ||
				!decode_fleet(record, stream.connection->peer, config_.numbers, fleet_hash_, status)) {
				return false;
			}
			stream.sequence = sequence;
			auto& observation = stream.connection->observation;
			if (observation.instance != stream.instance || sequence > observation.sequence) {
				observation = {status, stream.instance, sequence, Clock::now()};
				update_latency(*stream.connection);
			}
			consumed += 36;
		}
		bytes.erase(bytes.begin(), bytes.begin() + static_cast<std::ptrdiff_t>(consumed));
		return bytes.size() < 36;
	}
	/**
	 * @brief Queue an owned buffer; release it after acknowledgment/cancellation.
	 * @return False when the stream's pending-byte limit drops the sample.
	 */
	bool queue(Stream& stream, std::vector<uint8_t> bytes) {
		if (stream.pending + bytes.size() > config_.max_pending) {
			++send_drops_;
			warn("Dropping message: QUIC topic channel " + stream.key + " is backpressured");
			return false;
		}
		auto* buffer = new SendBuffer{std::move(bytes), {}};
		buffer->buffer = QUIC_BUFFER{static_cast<uint32_t>(buffer->data.size()), buffer->data.data()};
		stream.pending += buffer->data.size();
		const auto status = api_->StreamSend(stream.handle, &buffer->buffer, 1, QUIC_SEND_FLAG_NONE, buffer);
		if (QUIC_FAILED(status)) {
			stream.pending -= buffer->data.size();
			delete buffer;
			check(status, "StreamSend");
		}
		return true;
	}
	/**
	 * @brief Reassemble stream chunks into complete ROS messages, checking their sizes.
	 * Header: QCR1, sender(u16), boot ID(u64), key/type lengths(u16), key/type bytes.
	 * Repeated records: sequence(u64), CDR length(u32), CDR bytes. All integers are
	 * big-endian. Unknown keys, mismatched types and invalid sizes abort this stream.
	 */
	bool parse(Stream& stream) {
		auto& bytes = stream.input;
		if (stream.fleet || (!stream.header && bytes.size() >= 4 && std::memcmp(bytes.data(), "QCF2", 4) == 0)) {
			stream.fleet = true;
			return parse_fleet(stream);
		}
		if (!stream.header) {
			if (bytes.size() < 18) {
				return true;
			}
			const size_t key_length = integer(bytes.data() + 14, 2), type_length = integer(bytes.data() + 16, 2);
			if (std::memcmp(bytes.data(), "QCR1", 4) != 0 ||
				integer(bytes.data() + 4, 2) != static_cast<uint64_t>(stream.connection->peer) || key_length == 0 ||
				key_length > 1024 || type_length == 0 || type_length > 1024) {
				return false;
			}
			if (bytes.size() < 18 + key_length + type_length) {
				return true;
			}
			stream.key.assign(reinterpret_cast<const char*>(bytes.data() + 18), key_length);
			const std::string type(reinterpret_cast<const char*>(bytes.data() + 18 + key_length), type_length);
			const auto topic = std::find_if(config_.topics.begin(), config_.topics.end(),
				[&](const auto& entry) { return entry.key == stream.key && entry.type == type; });
			if (topic == config_.topics.end()) {
				return false;
			}
			stream.instance = integer(bytes.data() + 6, 8);
			stream.header = true;
			bytes.erase(bytes.begin(), bytes.begin() + static_cast<std::ptrdiff_t>(18 + key_length + type_length));
		}
		size_t consumed = 0;
		while (bytes.size() - consumed >= 12) {
			const auto sequence = integer(bytes.data() + consumed, 8);
			const size_t length = integer(bytes.data() + consumed + 8, 4);
			if (length < 4 || length > config_.max_message) {
				return false;
			}
			if (bytes.size() - consumed - 12 < length) {
				break;
			}
			while (receive_bytes_ + length > config_.max_pending && !received_.empty()) {
				receive_bytes_ -= received_.front().payload.size();
				const auto& dropped = received_.front();
				++receive_drops_by_topic_[{dropped.peer, dropped.key}];
				received_.pop_front();
				++receive_drops_;
				warn("ROS receive queue full; dropping oldest pending sample");
			}
			Received message{stream.connection->peer, stream.instance, sequence, stream.key, {}};
			message.payload.assign(bytes.begin() + static_cast<std::ptrdiff_t>(consumed + 12),
				bytes.begin() + static_cast<std::ptrdiff_t>(consumed + 12 + length));
			received_.push_back(std::move(message));
			receive_bytes_ += length;
			consumed += 12 + length;
		}
		bytes.erase(bytes.begin(), bytes.begin() + static_cast<std::ptrdiff_t>(consumed));
		return bytes.size() <= config_.max_message + 12;
	}
	/**
	 * @brief Check the certificate's UAV name and its saved fingerprint or certificate authority.
	 * Explicit CA mode uses MsQuic chain validation. Automatic mode checks the leaf
	 * signature and persisted TOFU pin. Both require matching common name and DNS SAN.
	 */
	int certificate_peer(QUIC_CERTIFICATE* certificate, int expected) {
		if (!certificate) {
			return 0;
		}
		const auto* buffer = reinterpret_cast<const QUIC_BUFFER*>(certificate);
		const unsigned char* bytes = buffer->Buffer;
		std::unique_ptr<X509, decltype(&X509_free)> x509(d2i_X509(nullptr, &bytes, buffer->Length), X509_free);
		if (!x509) {
			return 0;
		}
		std::array<char, 64> cn{};
		const int length =
			X509_NAME_get_text_by_NID(X509_get_subject_name(x509.get()), NID_commonName, cn.data(), cn.size());
		if (length < 4 || length > 5 ||
			!std::regex_match(std::string(cn.data(), length), std::regex("uav[0-9]{1,2}"))) {
			return 0;
		}
		const int peer = std::stoi(std::string(cn.data() + 3, length - 3));
		if (!config_.numbers.count(peer) || peer == config_.own || peer != expected ||
			config_.name(peer) != std::string(cn.data(), length) ||
			X509_check_host(x509.get(), cn.data(), length, X509_CHECK_FLAG_NO_WILDCARDS, nullptr) != 1) {
			return 0;
		}
		return certificates_.accept(x509.get(), peer) ? peer : 0;
	}
	/** @brief Accept the connection and attach the server certificate before the handshake. */
	static QUIC_STATUS QUIC_API listener_callback(HQUIC, void* context, QUIC_LISTENER_EVENT* event) {
		if (event->Type != QUIC_LISTENER_EVENT_NEW_CONNECTION) {
			return QUIC_STATUS_SUCCESS;
		}
		auto& listener = *static_cast<Listener*>(context);
		auto& self = *listener.transport;
		std::unique_lock<std::recursive_mutex> lock(self.mutex_);
		if (self.stopping_ || self.connections_.size() >= self.config_.numbers.size() * 4 + 8) {
			return QUIC_STATUS_CONNECTION_REFUSED;
		}
		auto connection = std::make_unique<Connection>();
		connection->transport = &self;
		connection->direct = listener.direct;
		// Restrict first-contact trust to the configured network identity before any
		// certificate can be pinned. Every incoming peer must be the lower numbered dialer.
		for (const int peer : self.config_.numbers) {
			const auto remote = self.address(peer, listener.direct, 0);
			if (peer < self.config_.own && event->NEW_CONNECTION.Info->RemoteAddress->Ip.sa_family == AF_INET &&
				remote.Ipv4.sin_addr.s_addr == event->NEW_CONNECTION.Info->RemoteAddress->Ipv4.sin_addr.s_addr) {
				connection->peer = peer;
				break;
			}
		}
		if (!connection->peer) {
			return QUIC_STATUS_CONNECTION_REFUSED;
		}
		connection->handle = event->NEW_CONNECTION.Connection;
		auto* state = connection.get();
		self.connections_.push_back(std::move(connection));
		self.api_->SetCallbackHandler(state->handle, reinterpret_cast<void*>(connection_callback), state);
		// SetParam can synchronize a worker; let callbacks acquire the state lock.
		// ListenerClose waits for this callback before stop releases any contexts.
		lock.unlock();
		// Apply fair stream scheduling on accepted connections as well as dialed
		// ones, so large topic channels cannot monopolize fleet heartbeats.
		const QUIC_STREAM_SCHEDULING_SCHEME scheduling = QUIC_STREAM_SCHEDULING_SCHEME_ROUND_ROBIN;
		if (QUIC_FAILED(self.api_->SetParam(
				state->handle, QUIC_PARAM_CONN_STREAM_SCHEDULING_SCHEME, sizeof(scheduling), &scheduling))) {
			self.api_->ConnectionShutdown(state->handle, QUIC_CONNECTION_SHUTDOWN_FLAG_NONE, 1);
			return QUIC_STATUS_SUCCESS;
		}
		if (state->direct) {
			const auto dscp = static_cast<uint8_t>(self.config_.direct_dscp);
			if (QUIC_FAILED(self.api_->SetParam(state->handle, QUIC_PARAM_CONN_SEND_DSCP, sizeof(dscp), &dscp))) {
				self.api_->ConnectionShutdown(state->handle, QUIC_CONNECTION_SHUTDOWN_FLAG_NONE, 1);
				return QUIC_STATUS_SUCCESS;
			}
		}
		const auto status = self.api_->ConnectionSetConfiguration(state->handle, self.server_config_);
		if (QUIC_FAILED(status)) {
			// Return success to retain ownership; never close a rejected NEW_CONNECTION twice.
			self.api_->ConnectionShutdown(state->handle, QUIC_CONNECTION_SHUTDOWN_FLAG_NONE, 1);
		}
		return QUIC_STATUS_SUCCESS;
	}
	/** @brief Authenticate peers, attach stream contexts and record connection lifecycle. */
	static QUIC_STATUS QUIC_API connection_callback(HQUIC, void* context, QUIC_CONNECTION_EVENT* event) {
		auto& connection = *static_cast<Connection*>(context);
		auto& self = *connection.transport;
		std::lock_guard<std::recursive_mutex> lock(self.mutex_);
		switch (event->Type) {
			case QUIC_CONNECTION_EVENT_PEER_CERTIFICATE_RECEIVED: {
				try {
					const int peer =
						self.certificate_peer(event->PEER_CERTIFICATE_RECEIVED.Certificate, connection.peer);
					if (!peer) {
						self.warn("Rejected unconfigured, changed or mismatched UAV certificate");
						return QUIC_STATUS_BAD_CERTIFICATE;
					}
				} catch (const std::exception& error) {
					self.warn(std::string("Certificate validation failed: ") + error.what());
					return QUIC_STATUS_BAD_CERTIFICATE;
				}
				break;
			}
			case QUIC_CONNECTION_EVENT_CONNECTED:
				if (!connection.peer) {
					return QUIC_STATUS_BAD_CERTIFICATE;
				}
				connection.connected = true;
				self.update_latency(connection);
				RCLCPP_INFO(self.logger_, "Connected to %s over %s (mutual TLS)",
					self.config_.name(connection.peer).c_str(), connection.direct ? "Wi-Fi Direct" : "AP");
				break;
			case QUIC_CONNECTION_EVENT_PEER_STREAM_STARTED: {
				auto stream = std::make_unique<Stream>();
				stream->connection = &connection;
				stream->handle = event->PEER_STREAM_STARTED.Stream;
				auto* state = stream.get();
				connection.streams.push_back(std::move(stream));
				self.api_->SetCallbackHandler(state->handle, reinterpret_cast<void*>(stream_callback), state);
				if (!(event->PEER_STREAM_STARTED.Flags & QUIC_STREAM_OPEN_FLAG_UNIDIRECTIONAL)) {
					self.api_->StreamShutdown(state->handle, QUIC_STREAM_SHUTDOWN_FLAG_ABORT, 1);
				}
				break;
			}
			case QUIC_CONNECTION_EVENT_SHUTDOWN_INITIATED_BY_TRANSPORT:
				connection.connected = false;
				self.warn("QUIC connection to " + self.config_.name(connection.peer) +
					(connection.direct ? " over Wi-Fi Direct" : " over AP") + " closed (" +
					std::to_string(event->SHUTDOWN_INITIATED_BY_TRANSPORT.Status) + ")");
				break;
			case QUIC_CONNECTION_EVENT_SHUTDOWN_INITIATED_BY_PEER:
				connection.connected = false;
				break;
			case QUIC_CONNECTION_EVENT_SHUTDOWN_COMPLETE:
				connection.connected = false;
				connection.finished = true;
				break;
			default:
				break;
		}
		return QUIC_STATUS_SUCCESS;
	}
	/** @brief Handle received data and free buffers when MsQuic finishes using them. */
	static QUIC_STATUS QUIC_API stream_callback(HQUIC handle, void* context, QUIC_STREAM_EVENT* event) {
		auto& stream = *static_cast<Stream*>(context);
		auto& self = *stream.connection->transport;
		std::lock_guard<std::recursive_mutex> lock(self.mutex_);
		switch (event->Type) {
			case QUIC_STREAM_EVENT_RECEIVE:
				for (uint32_t i = 0; i < event->RECEIVE.BufferCount; ++i) {
					const auto& buffer = event->RECEIVE.Buffers[i];
					stream.input.insert(stream.input.end(), buffer.Buffer, buffer.Buffer + buffer.Length);
					if (!self.parse(stream)) {
						self.warn("Rejected unknown, mismatched or malformed QUIC channel");
						stream.input.clear();
						self.api_->StreamShutdown(handle, QUIC_STREAM_SHUTDOWN_FLAG_ABORT, 1);
						break;
					}
				}
				break;
			case QUIC_STREAM_EVENT_SEND_COMPLETE: {
				const auto* buffer = static_cast<SendBuffer*>(event->SEND_COMPLETE.ClientContext);
				stream.pending -= buffer->data.size();
				delete buffer;
				break;
			}
			case QUIC_STREAM_EVENT_PEER_SEND_SHUTDOWN:
			case QUIC_STREAM_EVENT_PEER_SEND_ABORTED:
			case QUIC_STREAM_EVENT_PEER_RECEIVE_ABORTED:
				self.api_->StreamShutdown(handle, QUIC_STREAM_SHUTDOWN_FLAG_ABORT, 0);
				break;
			case QUIC_STREAM_EVENT_SHUTDOWN_COMPLETE:
				stream.handle = nullptr;
				self.api_->StreamClose(handle);
				break;
			default:
				break;
		}
		return QUIC_STATUS_SUCCESS;
	}
	/** @brief Stop listeners, synchronize connection callbacks, then free the MsQuic registration. */
	void stop() {
		if (!api_) {
			return;
		}
		{
			std::lock_guard<std::recursive_mutex> lock(mutex_);
			stopping_ = true;
		}
		for (auto* listener : {&ap_listener_, &direct_listener_}) {
			if (listener->handle) {
				api_->ListenerStop(listener->handle);
				api_->ListenerClose(listener->handle);
				listener->handle = nullptr;
			}
		}
		// ConnectionClose synchronizes callbacks and implicitly shuts down remaining streams.
		for (const auto& connection : connections_) {
			api_->ConnectionShutdown(connection->handle, QUIC_CONNECTION_SHUTDOWN_FLAG_SILENT, 0);
			api_->ConnectionClose(connection->handle);
		}
		connections_.clear();
		if (client_config_) {
			api_->ConfigurationClose(client_config_);
		}
		if (server_config_) {
			api_->ConfigurationClose(server_config_);
		}
		if (registration_) {
			api_->RegistrationClose(registration_);
		}
		MsQuicClose(api_);
		api_ = nullptr;
	}
	const Config& config_;
	rclcpp::Logger logger_;
	Certificates certificates_;
	const QUIC_API_TABLE* api_{};
	HQUIC registration_{}, server_config_{}, client_config_{};
	Listener ap_listener_, direct_listener_;
	std::recursive_mutex mutex_;
	bool stopping_{};
	uint64_t instance_{}, fleet_hash_{}, fleet_sequence_{};
	std::string direct_interface_;
	InterfaceNetwork direct_network_, ap_network_;
	std::vector<std::unique_ptr<Connection>> connections_;
	std::map<std::pair<int, bool>, Clock::time_point> next_connect_;
	Clock::time_point next_listener_{}, next_warning_{};
	size_t receive_bytes_{};
	uint64_t send_drops_{}, receive_drops_{};
	std::map<std::pair<int, std::string>, TopicTraffic> exported_;
	std::map<std::pair<int, std::string>, uint64_t> receive_drops_by_topic_;
	std::deque<Received> received_;
};

}  // namespace mrs_uav_wifi_direct
