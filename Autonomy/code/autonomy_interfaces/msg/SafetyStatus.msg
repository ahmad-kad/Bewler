# Safety Status Message
# Comprehensive safety system status with trigger types and recovery requirements

std_msgs/Header header

# Safety state
bool is_safe                     # Overall safety status
string safety_level              # "NORMAL", "WARNING", "CRITICAL", "EMERGENCY"
string[] active_triggers         # List of currently active safety triggers

# Trigger information
string trigger_type              # Type of safety trigger
string trigger_source            # Subsystem that triggered safety response
builtin_interfaces/Time trigger_time  # When safety was triggered
string trigger_description       # Human-readable description

# Recovery information
bool requires_manual_intervention  # True if manual recovery required
bool can_auto_recover            # True if automatic recovery possible
string[] recovery_steps          # List of steps needed for recovery
float64 estimated_recovery_time  # Estimated time to recover (seconds)

# Context-specific information
string context_state             # State when safety was triggered
string mission_phase             # Mission context when triggered
bool safe_to_retry               # True if safe to retry after recovery

# System health
float64 battery_level            # Current battery percentage
float64 temperature              # System temperature (Celsius)
bool communication_ok            # Communication link status
bool sensors_ok                  # Sensor system status
string[] degraded_capabilities   # List of capabilities that are degraded
