# AOIStatus.msg
# Age of Information status for sensor/system monitoring

std_msgs/Header header

# Sensor identification
string sensor_name         # Name of sensor/system (e.g., "imu", "gps", "slam_pose")
string sensor_type         # Type category (e.g., "sensor", "system", "fusion")

# AoI metrics (seconds)
float64 current_aoi        # Current age of information
float64 average_aoi        # Rolling average AoI over last N samples
float64 max_aoi           # Maximum AoI observed in window
float64 min_aoi           # Minimum AoI observed in window

# Quality assessment
bool is_fresh            # Whether current data is within acceptable age
float64 quality_score     # 0.0-1.0 quality score based on AoI
string freshness_status   # "FRESH", "ACCEPTABLE", "STALE", "CRITICAL"

# Configuration
float64 acceptable_threshold # Maximum acceptable AoI (seconds)
float64 optimal_threshold    # Optimal AoI threshold (seconds)

# Statistics
uint32 sample_count       # Number of samples in current window
float64 freshness_ratio   # Ratio of fresh samples (0.0-1.0)

# Network-aware metrics
string transport_type      # Transport type: "SERIAL", "CAN", "ETHERNET", "LOCAL"
float64 network_latency    # Network transmission latency (seconds)
float64 transport_latency  # Transport-specific latency (serial/CAN delays)
bool congestion_detected   # Whether network congestion was detected
float64 congestion_factor  # Current congestion multiplier (1.0 = no congestion)

# Predictive metrics
float64 predicted_aoi      # Predicted AOI for next sample
float64 aoi_trend          # AOI trend (-1 decreasing, 0 stable, 1 increasing)
