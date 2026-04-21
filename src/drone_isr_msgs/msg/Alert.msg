# Alert types: "TARGET_DETECTED", "ZONE_COVERED", "MISSION_COMPLETE"
string alert_type

# Target info (populated for TARGET_DETECTED alerts)
string target_label
float32 target_x
float32 target_y
float32 confidence

# Timestamp
builtin_interfaces/Time stamp
