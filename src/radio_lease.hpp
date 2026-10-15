// SPDX-License-Identifier: BSD-3-Clause
/** @file radio_lease.hpp
 * @brief Keep one Wi-Fi Direct manager per radio and local user.
 */
#pragma once

#include <fcntl.h>
#include <net/if.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace mrs_uav_wifi_direct {
/**
 * @brief Reject a second manager before it can change shared supplicant state.
 * Interface aliases on the same physical radio share one lock. The kernel releases
 * the lock after a crash, so no stale PID cleanup or administrator action is needed.
 * The file remains in place: unlinking it would allow another process to lock a
 * different inode while a concurrent opener still holds the original one.
 */
class RadioLease {
	public:
	/** @brief Take the user's radio lock, or report that another manager holds it. */
	explicit RadioLease(const std::string& interface) {
		std::ifstream source("/sys/class/net/" + interface + "/phy80211/name");
		std::string radio;
		source >> radio;
		if (radio.empty()) {
			// Still serialize retries when hardware is temporarily unavailable.
			radio = interface;
		}
		const auto path = "/tmp/mrs-uav-wifi-direct-" + std::to_string(::geteuid()) + "-" + radio + ".lock";
		fd_ = ::open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
		struct stat status{};
		if (fd_ < 0 || ::fstat(fd_, &status) != 0 || !S_ISREG(status.st_mode) || status.st_uid != ::geteuid() ||
			status.st_nlink != 1 || ::flock(fd_, LOCK_EX | LOCK_NB) != 0) {
			if (fd_ >= 0) {
				::close(fd_);
			}
			throw std::runtime_error("Cannot exclusively manage Wi-Fi radio " + radio +
				"; stop any other mrs_uav_wifi_direct node using this radio");
		}
	}
	/** @brief Release ownership only after the radio worker has stopped. */
	~RadioLease() {
		::close(fd_);
	}
	RadioLease(const RadioLease&) = delete;
	RadioLease& operator=(const RadioLease&) = delete;
	/**
	 * @brief Recover only an interface recorded by this manager in the current boot.
	 * The kernel interface index prevents a reused name from matching another
	 * process's group. The caller additionally verifies its radio and P2P role.
	 */
	std::string previous_group() const {
		char record[128]{};
		const auto size = ::pread(fd_, record, sizeof(record) - 1, 0);
		if (size <= 0) {
			return {};
		}
		std::string boot, interface, extra;
		unsigned index{};
		std::istringstream input(std::string(record, static_cast<size_t>(size)));
		if (!(input >> boot >> interface >> index) || (input >> extra) || boot != boot_id() ||
			interface.size() >= IFNAMSIZ || !index || ::if_nametoindex(interface.c_str()) != index) {
			return {};
		}
		return interface;
	}
	/** @brief Persist this group's identity before further initialization can fail or the process can die. */
	void remember(const std::string& interface) {
		const auto index = ::if_nametoindex(interface.c_str());
		if (!index) {
			throw std::runtime_error("Cannot record a disappeared Wi-Fi Direct interface");
		}
		const auto record = boot_id() + " " + interface + " " + std::to_string(index) + "\n";
		if (::pwrite(fd_, record.data(), record.size(), 0) != static_cast<ssize_t>(record.size()) ||
			::ftruncate(fd_, record.size()) != 0 || ::fsync(fd_) != 0) {
			throw std::runtime_error("Cannot record Wi-Fi Direct group ownership");
		}
	}
	/** @brief Forget ownership only after removal succeeds or the recorded interface no longer matches. */
	void forget() {
		if (::ftruncate(fd_, 0) != 0 || ::fsync(fd_) != 0) {
			throw std::runtime_error("Cannot clear Wi-Fi Direct group ownership");
		}
	}

	private:
	/** @brief Read the boot ID so recovery records apply to the current boot. */
	static std::string boot_id() {
		std::ifstream source("/proc/sys/kernel/random/boot_id");
		std::string value;
		source >> value;
		if (value.empty()) {
			throw std::runtime_error("Cannot identify this boot for Wi-Fi Direct recovery");
		}
		return value;
	}
	int fd_{-1};
};
}  // namespace mrs_uav_wifi_direct
