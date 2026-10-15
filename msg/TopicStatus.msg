# Cumulative local counters for one topic and peer, retained across reconnects.
string key
string source_topic
string received_topic
string message_type
# Zero means unlimited. Rates use elapsed wall time; NaN means no rate history yet.
float64 rate_limit_hz
bool source_reliable
bool source_present
float64 source_hz
float64 exported_hz
float64 imported_hz
# Exported samples were accepted by QUIC, before acknowledgment.
uint64 exported_messages
uint64 imported_messages
uint64 export_drops
uint64 import_drops
float64 source_age_s
float64 import_age_s
