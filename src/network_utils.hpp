// SPDX-License-Identifier: BSD-3-Clause
/** @file network_utils.hpp
 * @brief Read interface addresses and run Wi-Fi status queries.
 */
#pragma once

#include <arpa/inet.h>
#include <fcntl.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

extern char** environ;

namespace mrs_uav_wifi_direct {
/**
 * @brief An IPv4 address and subnet assigned to a host interface.
 * Each network follows the fleet's UAV address mapping. Infer peer addresses from
 * our assigned IPv4 address and UAV number, including on externally managed links.
 * Reject peers outside this subnet instead of sending them through another network.
 */
struct InterfaceNetwork {
	std::string interface, address;
	uint32_t ip{}, mask{}, index{};
	/** @brief Distinguish an assigned address from an absent/unaddressed interface. */
	explicit operator bool() const {
		return !address.empty();
	}
	/** @brief Rebind QUIC whenever interface identity, address or subnet changes. */
	bool operator==(const InterfaceNetwork& other) const {
		return interface == other.interface && ip == other.ip && mask == other.mask && index == other.index;
	}
	/** @brief Calculate a configured UAV's address within this subnet. */
	std::string peer_address(int own, int peer) const {
		const int64_t mapped = static_cast<int64_t>(ip) - own + peer;
		if (!*this || mapped < 0 || mapped > UINT32_MAX || (static_cast<uint32_t>(mapped) & mask) != (ip & mask) ||
			(static_cast<uint32_t>(mapped) & ~mask) == 0 || (static_cast<uint32_t>(mapped) & ~mask) == ~mask) {
			throw std::runtime_error("Interface subnet cannot address this UAV list using the host's UAV mapping");
		}
		in_addr value{htonl(static_cast<uint32_t>(mapped))};
		std::array<char, INET_ADDRSTRLEN> result{};
		::inet_ntop(AF_INET, &value, result.data(), result.size());
		return result.data();
	}
};

/** @brief Read whether the interface has a working link. */
inline bool interface_carrier(const std::string& interface) {
	std::ifstream input("/sys/class/net/" + interface + "/carrier");
	int carrier = 0;
	return (input >> carrier) && carrier == 1;
}

/** @brief Select an assigned unicast IPv4 network, excluding link-local fallback addresses. */
inline InterfaceNetwork interface_network(const std::string& interface) {
	ifaddrs* list{};
	if (interface.empty() || ::getifaddrs(&list) != 0) {
		return {};
	}
	InterfaceNetwork result;
	for (auto* item = list; item; item = item->ifa_next) {
		if (!item->ifa_addr || !item->ifa_netmask || item->ifa_addr->sa_family != AF_INET ||
			interface != item->ifa_name || !(item->ifa_flags & IFF_UP)) {
			continue;
		}
		const auto ip = ntohl(reinterpret_cast<sockaddr_in*>(item->ifa_addr)->sin_addr.s_addr);
		const auto mask = ntohl(reinterpret_cast<sockaddr_in*>(item->ifa_netmask)->sin_addr.s_addr);
		if (!mask || (ip >> 24) == 0 || (ip >> 24) >= 224 || (ip >> 16) == 0xa9fe ||
			((ip >> 24) == 127 && !(item->ifa_flags & IFF_LOOPBACK))) {
			continue;
		}
		std::array<char, INET_ADDRSTRLEN> address{};
		::inet_ntop(AF_INET, &reinterpret_cast<sockaddr_in*>(item->ifa_addr)->sin_addr, address.data(), address.size());
		result = {interface, address.data(), ip, mask, ::if_nametoindex(interface.c_str())};
		break;
	}
	::freeifaddrs(list);
	return result;
}

/**
 * @brief Run a command with separate arguments and capture its output.
 * Used for Wi-Fi queries and the network manager's configuration reload request.
 * A failed command raises an exception with its output. Commands inherit the
 * user's permissions; the existing network manager applies address configuration.
 */
inline std::string command(const std::vector<std::string>& args) {
	// Preserve the caller's existing permissions and avoid shell interpolation.
	int fds[2];
	if (::pipe2(fds, O_CLOEXEC) != 0) {
		throw std::runtime_error("pipe failed");
	}
	posix_spawn_file_actions_t actions;
	posix_spawn_file_actions_init(&actions);
	posix_spawn_file_actions_adddup2(&actions, fds[1], STDOUT_FILENO);
	posix_spawn_file_actions_adddup2(&actions, fds[1], STDERR_FILENO);
	posix_spawn_file_actions_addclose(&actions, fds[0]);
	posix_spawn_file_actions_addclose(&actions, fds[1]);
	std::vector<char*> argv;
	for (const auto& arg : args) {
		argv.push_back(const_cast<char*>(arg.c_str()));
	}
	argv.push_back(nullptr);
	pid_t pid{};
	const int status = ::posix_spawnp(&pid, argv[0], &actions, nullptr, argv.data(), environ);
	posix_spawn_file_actions_destroy(&actions);
	::close(fds[1]);
	if (status) {
		::close(fds[0]);
		throw std::runtime_error(args[0] + ": " + std::strerror(status));
	}
	std::string output;
	std::array<char, 4096> bytes;
	ssize_t count;
	while ((count = ::read(fds[0], bytes.data(), bytes.size())) != 0) {
		if (count > 0) {
			output.append(bytes.data(), static_cast<size_t>(count));
		} else if (errno != EINTR) {
			break;
		}
	}
	::close(fds[0]);
	int result{};
	while (::waitpid(pid, &result, 0) < 0 && errno == EINTR) {
	}
	if (!WIFEXITED(result) || WEXITSTATUS(result) != 0) {
		throw std::runtime_error(args[0] + " failed: " + output);
	}
	return output;
}

}  // namespace mrs_uav_wifi_direct
