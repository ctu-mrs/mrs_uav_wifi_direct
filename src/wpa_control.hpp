// SPDX-License-Identifier: BSD-3-Clause
/** @file wpa_control.hpp
 * @brief Send commands to wpa_supplicant and receive Wi-Fi events.
 */
#pragma once

#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>

#include <cstring>
#include <set>
#include <sstream>

#include "config.hpp"

namespace mrs_uav_wifi_direct {
/**
 * @brief Open a wpa_supplicant control socket and close it when this object is destroyed.
 * Requests and subscribed events use separate instances: event traffic cannot be
 * mistaken for a command reply. A private directory protects the local socket.
 */
class WpaControl {
	public:
	/** @brief Connect to the Wi-Fi service already running on the host. */
	explicit WpaControl(const std::string& path) {
		// A private directory prevents socket-path collisions and symlink replacement.
		std::array<char, 40> pattern{};
		std::strcpy(pattern.data(), "/tmp/mrs-wifi-XXXXXX");
		if (!::mkdtemp(pattern.data())) {
			throw std::runtime_error("mkdtemp failed");
		}
		directory_ = pattern.data();
		local_ = directory_ + "/control";
		sockaddr_un local{}, remote{};
		local.sun_family = remote.sun_family = AF_UNIX;
		if (path.size() >= sizeof(remote.sun_path)) {
			cleanup();
			throw std::runtime_error("Control path too long");
		}
		std::strcpy(local.sun_path, local_.c_str());
		std::strcpy(remote.sun_path, path.c_str());
		fd_ = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0);
		if (fd_ < 0 || ::bind(fd_, reinterpret_cast<sockaddr*>(&local), sizeof(local)) < 0 ||
			::connect(fd_, reinterpret_cast<sockaddr*>(&remote), sizeof(remote)) < 0) {
			const int code = errno;
			const auto error = std::string(std::strerror(code));
			cleanup();
			throw std::runtime_error("wpa_supplicant " + path + ": " + error +
				(code == EACCES || code == EPERM
						? "; install mrs-uav-wifi-direct-support and log in again for Wi-Fi control access"
						: ""));
		}
	}
	~WpaControl() {
		cleanup();
	}
	WpaControl(const WpaControl&) = delete;
	WpaControl& operator=(const WpaControl&) = delete;
	/**
	 * @brief Send a command and wait up to ten seconds for its reply.
	 * Full-MAC drivers may delay control processing during standalone discovery
	 * across several bands. These requests run on the radio worker, so waiting
	 * for a legitimate scan does not block ROS delivery or QUIC maintenance.
	 */
	std::string request(const std::string& cmd) {
		if (::send(fd_, cmd.data(), cmd.size(), 0) < 0) {
			throw std::runtime_error("Supplicant send failed");
		}
		std::string result;
		if (!receive(10000, result)) {
			throw std::runtime_error("Supplicant request timed out: " + cmd);
		}
		return result;
	}
	/** @brief Require an OK reply and retain the failed operation in the error. */
	void ok(const std::string& cmd) {
		if (request(cmd).compare(0, 2, "OK") != 0) {
			throw std::runtime_error("Supplicant rejected " + cmd);
		}
	}
	/** @brief Poll an attached event socket, returning empty when the wait expires. */
	std::string event(int timeout) {
		std::string result;
		receive(timeout, result);
		return result;
	}

	private:
	/**
	 * @brief Handle an empty datagram as a valid reply.
	 * STA-FIRST/STA-NEXT return an empty datagram at the end of the station table.
	 * Treat this as the end of the table, so an empty group stays available.
	 */
	bool receive(int timeout, std::string& result) {
		pollfd item{fd_, POLLIN, 0};
		const auto deadline = Clock::now() + std::chrono::milliseconds(timeout);
		int available;
		do {
			const auto remaining = std::max<int64_t>(
				0, std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now()).count());
			available = ::poll(&item, 1, static_cast<int>(remaining));
		} while (available < 0 && errno == EINTR && Clock::now() < deadline);
		if (available <= 0 || !(item.revents & POLLIN)) {
			return false;
		}
		std::array<char, 16384> bytes{};
		const auto count = ::recv(fd_, bytes.data(), bytes.size(), 0);
		if (count < 0) {
			throw std::runtime_error("Supplicant receive failed");
		}
		result.assign(bytes.data(), static_cast<size_t>(count));
		return true;
	}
	/** @brief Close the endpoint before unlinking its private socket directory. */
	void cleanup() {
		if (fd_ >= 0) {
			::close(fd_);
			fd_ = -1;
		}

		if (!local_.empty()) {
			::unlink(local_.c_str());
		}
		if (!directory_.empty()) {
			::rmdir(directory_.c_str());
		}
	}
	int fd_{-1};
	std::string directory_, local_;
};

/**
 * @brief Snapshot supplicant interface names without changing any network.
 * Comparing snapshots identifies only the temporary client created by our join;
 * pre-existing interfaces, including the primary AP, must never be configured.
 */
inline std::set<std::string> control_interfaces(WpaControl& control) {
	std::set<std::string> interfaces;
	std::istringstream reply(control.request("INTERFACES"));
	std::string name;
	while (reply >> name) {
		if (!std::regex_match(name, std::regex("[A-Za-z0-9_.-]{1,32}")) || name == "FAIL") {
			throw std::runtime_error("Supplicant returned an invalid interface list");
		}
		interfaces.insert(name);
	}
	return interfaces;
}

}  // namespace mrs_uav_wifi_direct
