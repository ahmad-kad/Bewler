.. _safety_configuration:

Safety System Configuration
============================

This guide covers all configuration options for the URC 2026 Mars Rover safety system, including parameter definitions, environment-specific settings, and optimization recommendations.

Configuration Architecture
==========================

Configuration Sources
----------------------

The safety system supports multiple configuration sources with this priority order:

1. **Launch File Parameters** (highest priority)
   - Runtime parameter overrides
   - Environment-specific settings
   - Test configuration parameters

2. **ROS2 Parameter Server**
   - Dynamic parameter updates
   - Runtime reconfiguration
   - Service-based parameter changes

3. **YAML Configuration Files**
   - Persistent configuration storage
   - Default parameter values
   - Structured configuration management

4. **Hardcoded Defaults** (lowest priority)
   - Failsafe parameter values
   - System stability guarantees
   - Conservative safety thresholds

Parameter Categories
--------------------

**Safety Thresholds**
   - Battery voltage and percentage limits
   - Temperature operating ranges
   - Sensor data validity bounds
   - Motion and velocity constraints

**Timing Parameters**
   - Timeout values for monitoring
   - Response time requirements
   - Update frequency settings
   - Communication delays

**System Behavior**
   - Monitoring level selection
   - Alert escalation rules
   - Recovery procedure settings
   - Automatic vs manual modes

**Resource Limits**
   - CPU and memory utilization
   - Network bandwidth allocation
   - Storage requirements
   - Performance constraints

Safety Watchdog Configuration
=============================

Core Monitoring Configuration
------------------------------

**Monitoring Levels**::

   watchdog:
     levels:
       - HEARTBEAT        # Basic system heartbeat monitoring
       - STATE_TRANSITIONS # State machine transition validation
       - SUBSYSTEM_HEALTH  # Individual subsystem health checks
       - SENSOR_INTEGRITY  # Sensor data quality monitoring

**Timeout Thresholds**::

   watchdog:
     heartbeat_timeout: 5.0         # Heartbeat timeout (seconds)
     state_transition_timeout: 30.0 # State transition timeout (seconds)
     subsystem_health_timeout: 10.0 # Subsystem health timeout (seconds)
     sensor_integrity_timeout: 2.0  # Sensor data timeout (seconds)

Safety Thresholds
-----------------

**Battery Safety**::

   watchdog:
     battery_critical_threshold: 10.0  # Critical battery level (%)
     battery_low_threshold: 20.0       # Low battery warning (%)

**Thermal Safety**::

   watchdog:
     temperature_critical_threshold: 85.0  # Critical temperature (°C)
     temperature_warning_threshold: 70.0   # Warning temperature (°C)

**System Behavior**::

   watchdog:
     enable_emergency_stop: true      # Enable automatic emergency stops
     enable_automatic_recovery: false # Enable automatic recovery procedures

Runtime Parameter Changes
-------------------------

**Dynamic Configuration Updates**::

   # Update battery threshold
   ros2 param set /safety_watchdog battery_critical_threshold 12.0

   # Change monitoring levels
   ros2 param set /safety_watchdog watchdog_levels '["HEARTBEAT", "SUBSYSTEM_HEALTH"]'

   # Adjust timeout values
   ros2 param set /safety_watchdog heartbeat_timeout 7.0

   # Enable automatic recovery
   ros2 param set /safety_watchdog enable_automatic_recovery true

Redundant Safety Monitor Configuration
======================================

Sensor Validation Limits
-------------------------

**IMU Safety Limits**::

   redundant_monitor:
     imu_accel_max: 50.0   # Maximum acceleration (m/s²)
     imu_gyro_max: 20.0    # Maximum angular velocity (rad/s)
     imu_orientation_tolerance: 0.1  # Orientation quaternion tolerance

**GPS Safety Limits**::

   redundant_monitor:
     gps_position_uncertainty_max: 10.0  # Maximum position uncertainty (m)
     gps_velocity_max: 5.0               # Maximum safe velocity (m/s)
     gps_hdop_max: 5.0                   # Maximum HDOP for valid fix

**Battery Safety Limits**::

   redundant_monitor:
     battery_critical: 10.0    # Critical battery level (%)
     battery_warning: 20.0     # Warning battery level (%)
     battery_voltage_min: 10.5 # Minimum voltage (V)
     battery_voltage_max: 12.8 # Maximum voltage (V)

**Thermal Safety Limits**::

   redundant_monitor:
     temperature_critical: 85.0   # Critical temperature (°C)
     temperature_warning: 70.0    # Warning temperature (°C)
     temperature_rate_max: 5.0    # Maximum temperature change rate (°C/min)

Communication Settings
----------------------

**Timeout Configuration**::

   redundant_monitor:
     communication_timeout: 5.0     # Communication timeout (seconds)
     sensor_timeout: 2.0            # Individual sensor timeout (seconds)
     consistency_check_interval: 2.0 # Consistency check frequency (seconds)

**Health Scoring**::

   redundant_monitor:
     health_score_threshold: 0.7    # Minimum acceptable health score
     consecutive_failures_threshold: 5  # Consecutive failures before failure
     data_quality_decay_rate: 0.1   # Health score decay per error

Emergency Response Coordinator Configuration
===========================================

Response Coordination Settings
-------------------------------

**Emergency Stop Configuration**::

   emergency_coordinator:
     emergency_ack_timeout: 2.0      # Emergency acknowledgment timeout (seconds)
     subsystem_stop_timeout: 5.0     # Subsystem stop timeout (seconds)
     stabilization_period: 10.0      # System stabilization time (seconds)
     recovery_ack_timeout: 30.0      # Recovery acknowledgment timeout (seconds)

**Recovery Settings**::

   emergency_coordinator:
     enable_automatic_recovery: false  # Enable automatic recovery procedures
     recovery_validation_timeout: 60.0 # Recovery validation timeout (seconds)
     max_recovery_attempts: 3          # Maximum recovery attempts

**Coordination Behavior**::

   emergency_coordinator:
     parallel_shutdown: true          # Shutdown subsystems in parallel
     acknowledge_required: true       # Require acknowledgment from subsystems
     status_broadcast_interval: 1.0   # Status broadcast frequency (seconds)

Subsystem Emergency Services
-----------------------------

**Service Configuration**::

   emergency_coordinator:
     subsystem_services:
       navigation: "/navigation/emergency_stop"
       computer_vision: "/computer_vision/emergency_stop"
       slam: "/slam/emergency_stop"
       manipulation: "/manipulation/emergency_stop"
       led_status: "/led_status/emergency_stop"
       sensor_bridge: "/sensor_bridge/emergency_stop"

Safety Dashboard Configuration
==============================

Alert Management Settings
-------------------------

**Alert Processing**::

   dashboard:
     max_active_alerts: 10              # Maximum active alerts
     alert_escalation_threshold: 3      # Alerts before escalation
     auto_resolve_timeout: 300.0        # Auto-resolve timeout (seconds)
     alert_deduplication_window: 30.0   # Alert deduplication time (seconds)

**Severity Thresholds**::

   dashboard:
     critical_alert_threshold: 5        # Critical alerts threshold
     warning_alert_threshold: 10        # Warning alerts threshold
     info_alert_threshold: 20           # Info alerts threshold

System Health Monitoring
------------------------

**Health Assessment**::

   dashboard:
     health_score_threshold: 0.7        # Minimum health score
     health_update_interval: 5.0        # Health update frequency (seconds)
     health_trend_window: 10            # Health trend analysis window (samples)

**Component Monitoring**::

   dashboard:
     monitored_components:
       - state_machine
       - safety_watchdog
       - redundant_safety_monitor
       - emergency_response_coordinator
       - navigation
       - computer_vision
       - sensor_bridge
       - battery_monitor
       - temperature_monitor

**Performance Monitoring**::

   dashboard:
     performance_update_interval: 1.0   # Performance update frequency (seconds)
     performance_history_length: 100    # Performance history buffer size
     performance_alert_threshold: 0.9   # Performance alert threshold

Safety Integration Tester Configuration
=======================================

Test Scenario Configuration
---------------------------

**Enabled Scenarios**::

   integration_testing:
     enabled_scenarios:
       - BATTERY_CRITICAL
       - COMMUNICATION_LOSS
       - THERMAL_WARNING
       - SENSOR_FAILURE
       - EMERGENCY_STOP
       - RECOVERY_TEST
       - CONSISTENCY_CHECK
       - PERFORMANCE_LOAD

**Test Execution Settings**::

   integration_testing:
     test_timeout: 120.0        # Maximum test execution time (seconds)
     inter_test_delay: 5.0      # Delay between tests (seconds)
     validation_timeout: 30.0   # Validation phase timeout (seconds)
     concurrent_tests_max: 3    # Maximum concurrent test executions

Test Validation Criteria
------------------------

**Success Thresholds**::

   integration_testing:
     success_threshold: 0.8          # Minimum success rate for test suite
     timing_tolerance: 0.1           # Timing tolerance for validation (±10%)
     data_tolerance: 0.05            # Data tolerance for validation (±5%)

**Failure Handling**::

   integration_testing:
     max_test_failures: 3            # Maximum test failures before abort
     failure_retry_count: 2          # Number of failure retries
     failure_retry_delay: 10.0       # Delay between failure retries (seconds)

Test Data Configuration
-----------------------

**Sensor Simulation**::

   integration_testing:
     sensor_simulation:
       battery_discharge_rate: 1.0    # Battery discharge rate (%/second)
       temperature_rise_rate: 2.0    # Temperature rise rate (°C/second)
       imu_noise_level: 0.01         # IMU noise level (m/s², rad/s)
       gps_position_noise: 0.1       # GPS position noise (meters)

**Communication Simulation**::

   integration_testing:
     communication_simulation:
       packet_loss_rate: 0.01        # Simulated packet loss rate
       latency_variation: 0.05       # Latency variation (±50ms)
       connection_drop_duration: 5.0 # Connection drop duration (seconds)

Environment-Specific Configurations
===================================

Development Environment
-----------------------

**Relaxed Safety Thresholds**::

   # Development configuration - more permissive for testing
   development:
     safety_system:
       watchdog:
         battery_critical_threshold: 5.0    # Lower threshold for testing
         temperature_critical_threshold: 90.0  # Higher temperature limit
         heartbeat_timeout: 10.0            # Longer timeout for debugging

       redundant_monitor:
         battery_critical: 5.0              # Lower battery threshold
         temperature_critical: 90.0         # Higher temperature limit

       dashboard:
         max_active_alerts: 20              # Allow more alerts during development

       integration_testing:
         enabled_scenarios: ["BATTERY_CRITICAL", "EMERGENCY_STOP", "RECOVERY_TEST"]

**Debug Settings**::

   development:
     logging:
       level: DEBUG
       enable_verbose_output: true
       log_safety_events: true
       log_performance_metrics: true

     monitoring:
       enable_detailed_diagnostics: true
       performance_monitoring: true
       memory_usage_tracking: true

Competition Environment
-----------------------

**Strict Safety Thresholds**::

   # Competition configuration - strict safety requirements
   competition:
     safety_system:
       watchdog:
         battery_critical_threshold: 15.0   # URC minimum battery level
         temperature_critical_threshold: 75.0  # Competition temperature limit
         heartbeat_timeout: 3.0             # Shorter timeout for responsiveness

       redundant_monitor:
         battery_critical: 15.0             # Competition battery safety
         temperature_critical: 75.0         # Competition thermal safety
         communication_timeout: 2.0         # Faster communication detection

       emergency_coordinator:
         enable_automatic_recovery: false   # Manual recovery for competition
         parallel_shutdown: true            # Fast parallel shutdown

       dashboard:
         max_active_alerts: 5               # Limit alerts during competition
         alert_escalation_threshold: 2      # Faster escalation

**URC Compliance Settings**::

   competition:
     urc_requirements:
       emergency_stop_response_time: 3.0   # URC 3-second requirement
       emergency_stop_accessibility: 3.0   # 3-meter accessibility requirement
       safety_system_redundancy: true      # Redundant safety systems required
       manual_emergency_override: true     # Manual emergency stop required

     validation:
       pre_run_safety_check: true          # Pre-run safety validation
       continuous_monitoring: true         # Continuous safety monitoring
       post_run_analysis: true             # Post-run safety analysis

Simulation Environment
----------------------

**Simulation-Specific Settings**::

   # Simulation configuration - optimized for Gazebo simulation
   simulation:
     safety_system:
       watchdog:
         heartbeat_timeout: 2.0            # Faster timeout for simulation
         sensor_integrity_timeout: 1.0     # Faster sensor validation

       redundant_monitor:
         communication_timeout: 1.0        # Faster communication detection
         sensor_timeout: 0.5               # Faster sensor timeout

       integration_testing:
         enabled_scenarios: ["ALL"]         # Enable all test scenarios
         test_timeout: 60.0                # Shorter test timeout
         success_threshold: 0.95           # Higher success requirement

**Gazebo Integration**::

   simulation:
     gazebo:
       physics_time_step: 0.001            # High-frequency physics
       real_time_factor: 1.0               # Real-time simulation
       safety_plugin_enabled: true         # Enable safety plugins

     sensor_simulation:
       noise_enabled: true                 # Enable sensor noise
       failure_simulation: false           # Disable random failures (default)
       perfect_sensors: false              # Use realistic sensor models

Testing Environment
-------------------

**Comprehensive Testing Configuration**::

   # Testing configuration - maximum safety validation
   testing:
     safety_system:
       watchdog:
         levels: ["HEARTBEAT", "STATE_TRANSITIONS", "SUBSYSTEM_HEALTH", "SENSOR_INTEGRITY"]

       redundant_monitor:
         # Strict validation for testing
         imu_accel_max: 30.0               # Lower limits for testing
         battery_critical: 5.0             # Very low for test triggering

       dashboard:
         max_active_alerts: 50             # Allow many alerts during testing
         alert_escalation_threshold: 1     # Fast escalation for testing

       integration_testing:
         enabled_scenarios: ["ALL"]         # Enable all test scenarios
         test_timeout: 300.0               # Long timeout for comprehensive testing
         success_threshold: 0.9            # High success requirement
         concurrent_tests_max: 1           # Serial testing for accuracy

**Test Data Generation**::

   testing:
     test_data:
       battery_discharge_profiles: ["FAST", "MEDIUM", "SLOW"]
       temperature_profiles: ["LINEAR_RISE", "STEP_CHANGE", "THERMAL_RUNAWAY"]
       sensor_failure_modes: ["STUCK", "NOISE", "BIAS", "DROP_OUT"]
       communication_scenarios: ["PACKET_LOSS", "LATENCY", "CONNECTION_DROP"]

Configuration Validation
========================

Configuration File Validation
-----------------------------

**YAML Syntax Validation**::

   #!/bin/bash
   # Validate safety system configuration files

   CONFIG_FILES=(
     "config/safety_system.yaml"
     "config/development_safety.yaml"
     "config/competition_safety.yaml"
   )

   for config_file in "${CONFIG_FILES[@]}"; do
       echo "Validating $config_file..."

       # Check YAML syntax
       if python3 -c "import yaml; yaml.safe_load(open('$config_file'))"; then
           echo "✓ $config_file syntax valid"
       else
           echo "✗ $config_file syntax invalid"
           exit 1
       fi

       # Validate required sections
       if grep -q "safety_system:" "$config_file"; then
           echo "✓ $config_file contains safety_system section"
       else
           echo "✗ $config_file missing safety_system section"
           exit 1
       fi
   done

**Parameter Range Validation**::

   #!/bin/bash
   # Validate parameter ranges

   validate_range() {
       local value=$1
       local min=$2
       local max=$3
       local param=$4

       if (( $(echo "$value < $min" | bc -l) )) || (( $(echo "$value > $max" | bc -l) )); then
           echo "✗ $param value $value out of range [$min, $max]"
           return 1
       else
           echo "✓ $param value $value within range"
           return 0
       fi
   }

   # Validate critical parameters
   validate_range "$(ros2 param get /safety_watchdog battery_critical_threshold)" 5.0 20.0 "battery_critical_threshold"
   validate_range "$(ros2 param get /safety_watchdog temperature_critical_threshold)" 70.0 95.0 "temperature_critical_threshold"

Runtime Configuration Validation
--------------------------------

**Parameter Consistency Checks**::

   #!/bin/bash
   # Validate parameter consistency

   # Check that warning thresholds are above critical thresholds
   battery_critical=$(ros2 param get /safety_watchdog battery_critical_threshold)
   battery_warning=$(ros2 param get /safety_watchdog battery_warning_threshold)

   if (( $(echo "$battery_warning <= $battery_critical" | bc -l) )); then
       echo "✗ Battery warning threshold must be above critical threshold"
       exit 1
   else
       echo "✓ Battery threshold hierarchy correct"
   fi

**System Health Validation**::

   #!/bin/bash
   # Validate system health after configuration changes

   echo "Validating system health after configuration..."

   # Check that safety system is running
   if ros2 node list | grep -q "safety_watchdog"; then
       echo "✓ Safety watchdog node running"
   else
       echo "✗ Safety watchdog node not found"
       exit 1
   fi

   # Check that safety topics are published
   if ros2 topic list | grep -q "/safety/dashboard_status"; then
       echo "✓ Safety dashboard topic available"
   else
       echo "✗ Safety dashboard topic not found"
       exit 1
   fi

   # Validate configuration loading
   dashboard_status=$(ros2 topic echo /safety/dashboard_status --once --timeout 5 2>/dev/null)
   if [[ -n "$dashboard_status" ]]; then
       echo "✓ Safety system configuration loaded successfully"
   else
       echo "✗ Safety system configuration validation failed"
       exit 1
   fi

Configuration Backup and Recovery
=================================

Configuration Backup
--------------------

**Automated Backup**::

   #!/bin/bash
   # Backup safety system configuration

   BACKUP_DIR="/var/backups/safety_system"
   TIMESTAMP=$(date +%Y%m%d_%H%M%S)

   mkdir -p "$BACKUP_DIR"

   # Backup parameter values
   ros2 param dump /safety_watchdog > "$BACKUP_DIR/watchdog_params_$TIMESTAMP.yaml"
   ros2 param dump /redundant_safety_monitor > "$BACKUP_DIR/redundant_params_$TIMESTAMP.yaml"
   ros2 param dump /emergency_response_coordinator > "$BACKUP_DIR/coordinator_params_$TIMESTAMP.yaml"
   ros2 param dump /safety_dashboard > "$BACKUP_DIR/dashboard_params_$TIMESTAMP.yaml"

   # Backup configuration files
   cp config/safety_system.yaml "$BACKUP_DIR/safety_system_$TIMESTAMP.yaml"

   echo "Safety system configuration backed up to $BACKUP_DIR"

**Version Control Integration**::

   # Include configuration in version control
   git add config/safety_system.yaml
   git commit -m "Update safety system configuration

   Changes:
   - Updated battery critical threshold to 12%
   - Adjusted temperature warning threshold to 75°C
   - Enabled automatic recovery procedures

   Validation:
   - Configuration syntax validated
   - Parameter ranges verified
   - System health confirmed"

Configuration Recovery
----------------------

**Parameter Recovery**::

   #!/bin/bash
   # Recover safety system parameters from backup

   BACKUP_FILE="$1"

   if [[ ! -f "$BACKUP_FILE" ]]; then
       echo "Backup file not found: $BACKUP_FILE"
       exit 1
   fi

   echo "Recovering safety system parameters from $BACKUP_FILE..."

   # Load parameter backup
   ros2 param load /safety_watchdog "$BACKUP_FILE"
   ros2 param load /redundant_safety_monitor "$BACKUP_FILE"
   ros2 param load /emergency_response_coordinator "$BACKUP_FILE"
   ros2 param load /safety_dashboard "$BACKUP_FILE"

   echo "Safety system parameters recovered"

**Configuration Rollback**::

   #!/bin/bash
   # Rollback to previous configuration version

   ROLLBACK_VERSION="$1"

   if [[ -z "$ROLLBACK_VERSION" ]]; then
       echo "Usage: $0 <version>"
       echo "Available versions:"
       ls /var/backups/safety_system/ | grep safety_system_ | sort -r
       exit 1
   fi

   BACKUP_FILE="/var/backups/safety_system/safety_system_$ROLLBACK_VERSION.yaml"

   if [[ ! -f "$BACKUP_FILE" ]]; then
       echo "Rollback version not found: $ROLLBACK_VERSION"
       exit 1
   fi

   echo "Rolling back to configuration version $ROLLBACK_VERSION..."

   # Stop safety system
   ros2 lifecycle set /safety_watchdog shutdown
   ros2 lifecycle set /redundant_safety_monitor shutdown

   # Restore configuration
   cp "$BACKUP_FILE" config/safety_system.yaml

   # Restart safety system
   ros2 launch autonomy_safety_system safety_system.launch.py

   echo "Configuration rollback complete"

Performance Optimization
========================

Resource Optimization
---------------------

**CPU Optimization**::

   # Low-CPU configuration for resource-constrained systems
   low_power:
     safety_system:
       watchdog:
         health_check_interval: 5.0       # Reduce health check frequency
         monitoring_levels: ["HEARTBEAT", "SUBSYSTEM_HEALTH"]  # Reduce monitoring scope

       redundant_monitor:
         consistency_check_interval: 5.0  # Reduce consistency checks
         sensor_validation_rate: 5.0      # Reduce sensor validation frequency

       dashboard:
         update_interval: 5.0             # Reduce dashboard updates
         health_update_interval: 10.0     # Reduce health monitoring

**Memory Optimization**::

   # Low-memory configuration
   embedded:
     safety_system:
       dashboard:
         alert_history_size: 50           # Reduce alert history
         health_history_size: 20          # Reduce health history

       integration_testing:
         test_result_retention: 10        # Reduce test result retention

**Network Optimization**::

   # Low-bandwidth configuration
   bandwidth_constrained:
     safety_system:
       watchdog:
         heartbeat_interval: 2.0          # Reduce heartbeat frequency

       dashboard:
         status_update_interval: 5.0      # Reduce status updates
         diagnostics_interval: 30.0       # Reduce diagnostics frequency

Timing Optimization
-------------------

**Real-time Performance**::

   # High-performance configuration for real-time requirements
   real_time:
     safety_system:
       watchdog:
         emergency_stop_priority: 99      # High process priority
         real_time_scheduling: true       # Enable real-time scheduling

       emergency_coordinator:
         parallel_processing: true        # Enable parallel emergency processing
         optimized_timeouts: true         # Use optimized timeout values

**Latency Optimization**::

   # Low-latency configuration for fast response requirements
   low_latency:
     safety_system:
       watchdog:
         detection_interval: 0.01         # 100Hz detection
         processing_threads: 4            # Multi-threaded processing

       redundant_monitor:
         validation_threads: 2            # Parallel validation
         fast_path_enabled: true          # Enable fast validation path

This comprehensive configuration guide ensures the safety system can be properly configured for any environment while maintaining safety and performance requirements.
