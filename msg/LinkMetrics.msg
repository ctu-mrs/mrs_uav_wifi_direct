# A local station-table entry or an authenticated QUIC path.
# Radio counters describe the entire station link, including traffic from other programs.
# Floating-point measurements are NaN when unavailable. Missing counters are UINT64_MAX.
# Empty strings and zero frequency/width mean unknown.
string interface
string address
string mac_address
string ssid
bool connected
uint32 frequency_mhz
uint16 channel_width_mhz
float64 signal_dbm
float64 signal_average_dbm
float64 tx_bitrate_mbps
float64 rx_bitrate_mbps
float64 expected_throughput_mbps
float64 inactive_s
uint64 tx_packets
uint64 rx_packets
uint64 tx_retries
uint64 tx_failed
uint64 rx_dropped
# QUIC RTT comes from acknowledgments; this includes network queuing and processing.
float64 rtt_ms
float64 minimum_rtt_ms
float64 maximum_rtt_ms
float64 rtt_variation_ms
float64 rtt_age_s
