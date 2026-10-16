// SPDX-License-Identifier: BSD-3-Clause
/** @file p2p_address.hpp
 * @brief Validate the P2P subnet and prepare its networkd address before group creation.
 */
#pragma once

#include <sys/stat.h>

#include "network_utils.hpp"

namespace mrs_uav_wifi_direct {
/**
 * @brief A private IPv4 subnet shared by the fleet's automatically managed groups.
 * UAV n uses host offset 100+n. A /24 or larger private subnet accommodates every
 * supported UAV name. The support package supplies a writable runtime drop-in;
 * networkd assigns the address once the P2P group has completed authentication.
 */
class P2pSubnet {
	public:
	/** @brief Require a canonical private IPv4 network with room for all UAV addresses. */
	explicit P2pSubnet(const std::string& cidr = "192.168.255.0/24") : cidr_(cidr) {
		const auto slash = cidr.find('/');
		in_addr address{};
		if (slash == std::string::npos || ::inet_pton(AF_INET, cidr.substr(0, slash).c_str(), &address) != 1) {
			throw std::runtime_error("network.p2p_subnet must be a private IPv4 network in CIDR notation");
		}
		const auto bits = cidr.substr(slash + 1);
		if (bits.empty() || bits.size() > 2 || bits.find_first_not_of("0123456789") != std::string::npos) {
			throw std::runtime_error("network.p2p_subnet requires a prefix length from 8 to 24");
		}
		prefix_ = std::stoi(bits);
		if (prefix_ < 8 || prefix_ > 24 || std::to_string(prefix_) != bits) {
			throw std::runtime_error("network.p2p_subnet requires a prefix length from 8 to 24");
		}
		network_ = ntohl(address.s_addr);
		mask_ = UINT32_MAX << (32 - prefix_);
		const bool private_network = (network_ >> 24) == 10 || (prefix_ >= 12 && (network_ >> 20) == 0xac1) ||
			(prefix_ >= 16 && (network_ >> 16) == 0xc0a8);
		if (!private_network || (network_ & mask_) != network_) {
			throw std::runtime_error(
				"network.p2p_subnet must be an aligned network within 10/8, 172.16/12 or 192.168/16");
		}
	}
	/** @brief Return the validated setting for status and fleet configuration checks. */
	const std::string& cidr() const {
		return cidr_;
	}
	/** @brief Calculate the same address on every host without depending on its AP. */
	std::string address(int uav) const {
		if (uav < 1 || uav > 99) {
			throw std::runtime_error("A P2P address requires a UAV number from 1 to 99");
		}
		in_addr value{htonl(network_ + 100 + uav)};
		std::array<char, INET_ADDRSTRLEN> text{};
		::inet_ntop(AF_INET, &value, text.data(), text.size());
		return std::string(text.data()) + "/" + std::to_string(prefix_);
	}
	/** @brief Confirm that networkd applied both the requested address and prefix. */
	bool matches(const InterfaceNetwork& network, int uav) const {
		return network && network.mask == mask_ && network.address + "/" + std::to_string(prefix_) == address(uav);
	}
	/**
	 * @brief Validate the host networks and leave new P2P interfaces without an IP address.
	 * Reject overlap with existing host networks and groups managed elsewhere.
	 * WPS creates a temporary client interface. Giving it an address lets native
	 * IP traffic interrupt enrollment, so activate() waits for the completed group.
	 */
	void prepare(const std::string& interface, const std::string& previous_group) const {
		ifaddrs* list{};
		if (::getifaddrs(&list) != 0) {
			throw std::runtime_error("Cannot inspect interface subnets before P2P address setup");
		}
		std::string conflict;
		const auto group_prefix = "p2p-" + interface + "-";
		for (auto* item = list; item; item = item->ifa_next) {
			const std::string name = item->ifa_name;
			if (name != previous_group && name.compare(0, group_prefix.size(), group_prefix) == 0) {
				conflict = "Another manager owns " + name + "; select existing_group_interface or stop that manager";
				break;
			}
			if (!item->ifa_addr || !item->ifa_netmask || item->ifa_addr->sa_family != AF_INET ||
				previous_group == item->ifa_name) {
				continue;
			}
			const auto ip = ntohl(reinterpret_cast<sockaddr_in*>(item->ifa_addr)->sin_addr.s_addr);
			const auto mask = ntohl(reinterpret_cast<sockaddr_in*>(item->ifa_netmask)->sin_addr.s_addr);
			const auto common_mask = mask & mask_;
			if ((ip & common_mask) == (network_ & common_mask)) {
				conflict = "network.p2p_subnet overlaps the existing network on " + name;
				break;
			}
		}
		::freeifaddrs(list);
		if (!conflict.empty()) {
			throw std::runtime_error(conflict);
		}
		write_address("");
	}
	/** @brief Assign the configured address after WPS and WPA authentication finish. */
	void activate(int uav) const {
		write_address(address(uav));
	}

	private:
	/**
	 * @brief Update the package's runtime drop-in and ask networkd to apply it.
	 * The directory and match profile remain administrator-owned. Reload even an
	 * identical file so a retry recovers an interrupted or unsuccessful reload.
	 */
	void write_address(const std::string& value) const {
		const std::string path = "/run/systemd/network/10-mrs-wifi-direct.network.d/50-address.conf";
		const int fd = ::open(path.c_str(), O_RDWR | O_CLOEXEC | O_NOFOLLOW);
		struct stat info{};
		if (fd < 0) {
			throw std::runtime_error(
				"Cannot open the P2P address settings; install mrs-uav-wifi-direct-support and log in again");
		}
		try {
			if (::fstat(fd, &info) != 0 || !S_ISREG(info.st_mode) || info.st_uid != 0 || info.st_nlink != 1 ||
				(info.st_mode & S_IWOTH)) {
				throw std::runtime_error("Unexpected P2P address file ownership or permissions");
			}
			const auto contents = std::string("[Network]\n") + (value.empty() ? "" : "Address=" + value + "\n");
			std::array<char, 512> current{};
			const auto size = ::pread(fd, current.data(), current.size(), 0);
			if (size < 0) {
				throw std::runtime_error("Cannot read the P2P address settings");
			}
			if (size != static_cast<ssize_t>(contents.size()) || std::string(current.data(), size) != contents) {
				if (::pwrite(fd, contents.data(), contents.size(), 0) != static_cast<ssize_t>(contents.size()) ||
					::ftruncate(fd, contents.size()) != 0 || ::fsync(fd) != 0) {
					throw std::runtime_error("Cannot write the P2P address settings");
				}
			}
			command({"busctl", "--allow-interactive-authorization=no", "call", "org.freedesktop.network1",
				"/org/freedesktop/network1", "org.freedesktop.network1.Manager", "Reload"});
		} catch (...) {
			::close(fd);
			throw;
		}
		::close(fd);
	}

	std::string cidr_;
	uint32_t network_{}, mask_{};
	int prefix_{};
};
}  // namespace mrs_uav_wifi_direct
