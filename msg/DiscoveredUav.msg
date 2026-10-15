# A nearby public Wi-Fi advertisement, an authenticated peer, or a configured peer.
# Advertisement names and MAC addresses are unverified, even when configured=true.
# Only authenticated_* and fleet_* fields come from an authenticated QUIC connection.
string hostname
bool configured
bool advertisement_present
string device_address
string interface_address
string group_ssid
bool group_owner
uint32 frequency_mhz
float64 signal_dbm
float64 advertisement_age_s
bool authenticated_ap
bool authenticated_direct
float64 heartbeat_age_s
string fleet_role
string fleet_owner
uint16 fleet_members
bool fleet_ready
bool fleet_can_host
bool fleet_can_join
LinkMetrics ap
LinkMetrics direct
TopicStatus[] topics
