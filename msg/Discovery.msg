# Periodic snapshot of local radio state and nearby UAV observations.
# header.stamp is publication time; radio_age_s measures snapshot age with a steady clock.
# A paused ROS clock does not pause collection or publishing.
std_msgs/Header header
string hostname
string interface
string driver
string phy
bool communication_enabled
bool radio_available
float64 radio_age_s
string radio_error
string discovery_error
string direct_error
string group_interface
string group_role
string group_owner
bool group_ready
# AP measurements describe the local AP link, not a radio link to a remote UAV.
LinkMetrics ap
# Associated stations on local P2P group interfaces. Each includes its interface and MAC.
LinkMetrics[] direct_stations
# Includes other fleets and nearby unconfigured UAVs. Public observations expire at peer_timeout_s.
DiscoveredUav[] uavs
