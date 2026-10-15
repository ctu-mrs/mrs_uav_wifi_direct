// SPDX-License-Identifier: BSD-3-Clause
/** @file mrs_uav_wifi_direct.cpp
 * @brief Process entry point; helper headers beside this file contain the bridge components.
 */
#include "ros_bridge.hpp"

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	int result{};
	try {
		rclcpp::spin(std::make_shared<mrs_uav_wifi_direct::WifiDirectNode>());
	} catch (const std::exception& error) {
		RCLCPP_ERROR(rclcpp::get_logger("mrs_uav_wifi_direct"), "%s", error.what());
		result = 1;
	}
	rclcpp::shutdown();
	return result;
}
