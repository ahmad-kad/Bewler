# URC 2026 Communication Stress Testing Framework

This framework provides comprehensive stress testing for the URC 2026 rover's communication systems under conditions harsher than real-world scenarios.

## Overview

The stress testing framework evaluates system resilience across three critical communication layers:

- **Network Communication**: ROS2 DDS under extreme network conditions
- **CAN Bus Communication**: CAN bus faults, overload, and electrical issues
- **Movement Control**: Rapid command changes, conflicts, and emergency scenarios
- **Integrated Systems**: Combined stress across all communication layers

## Test Severity Levels

### Moderate Stress
- Network: 2% packet loss, 25ms latency
- CAN: 3% faults, 70% bus load
- Movement: 5% faults, basic conflict detection
- Duration: ~30 seconds per component

### Severe Stress
- Network: 8% packet loss, 100ms latency, 50Mbps bandwidth limit
- CAN: 12% faults, 90% bus load, arbitration conflicts
- Movement: 15% faults, 8% emergency stops, 10% conflicts
- Duration: ~60 seconds per component

### Extreme Stress
- Network: 25% packet loss, 300ms latency, 10Mbps bandwidth limit
- CAN: 35% faults, 130% bus overload, electrical noise
- Movement: 40% faults, 25% emergency stops, 30% conflicts
- Duration: ~120 seconds per component

## Quick Start

### Run Individual Stress Tests

```bash
# Network stress testing
cd /home/ubuntu/urc-machiato-2026
python3 tests/performance/stress_test_network_communication.py

# CAN bus stress testing
python3 tests/performance/stress_test_can_communication.py

# Movement control stress testing
python3 tests/performance/stress_test_movement_control.py

# Integrated system stress testing
python3 tests/performance/stress_test_integrated_system.py
```

### Run Complete Stress Test Suite

```bash
# Run all stress tests with comprehensive analysis
python3 tests/performance/stress_test_orchestrator.py

# Run with custom output directory
python3 tests/performance/stress_test_orchestrator.py --output-dir my_results

# Run quick test with reduced duration
python3 tests/performance/stress_test_orchestrator.py --quick-test
```

## Test Architecture

### Network Stress Test (`stress_test_network_communication.py`)
- **Purpose**: Test ROS2 DDS communication under extreme network conditions
- **Stressors**: Packet loss, latency, bandwidth limits
- **Metrics**: Latency, throughput, packet loss rate
- **Tools**: Linux `tc` (traffic control) for network emulation

### CAN Bus Stress Test (`stress_test_can_communication.py`)
- **Purpose**: Test CAN bus communication with bus faults and overload
- **Stressors**: Bus overload, arbitration conflicts, electrical faults
- **Metrics**: Bus availability, error rates, arbitration collisions
- **Simulation**: Software-based CAN bus fault injection

### Movement Control Stress Test (`stress_test_movement_control.py`)
- **Purpose**: Test rover movement control under rapid command changes
- **Stressors**: Command conflicts, emergency stops, rapid direction changes
- **Metrics**: Command success rate, response latency, conflict detection
- **ROS2 Integration**: Real publishers/subscribers with QoS profiles

### Integrated Stress Test (`stress_test_integrated_system.py`)
- **Purpose**: Test complete system under combined communication stress
- **Stressors**: All communication layers stressed simultaneously
- **Metrics**: System health score, resource usage, cross-system conflicts
- **Concurrency**: Multi-threaded stress testing

## Performance Metrics

### Network Performance
- **Latency**: Round-trip message latency (target: <50ms)
- **Throughput**: Messages per second (target: >1000 msg/s)
- **Packet Loss**: Percentage of lost messages (target: <5%)

### CAN Bus Performance
- **Bus Availability**: Percentage of time bus is operational (target: >95%)
- **Error Rate**: CAN bus errors per second (target: <5%)
- **Arbitration Success**: Successful message arbitration (target: >98%)

### Movement Control Performance
- **Command Success Rate**: Percentage of commands executed (target: >90%)
- **Response Latency**: Time to process movement commands (target: <20ms)
- **Conflict Resolution**: Successful handling of conflicting commands

### System Health
- **Overall Health Score**: Composite score 0-100 (target: >75)
- **Resource Usage**: CPU/memory usage under stress (target: <80%)
- **Recovery Time**: Time to recover from faults (target: <5s)

## Understanding Test Results

### Health Score Interpretation
- **80-100**: ✅ Excellent - System handles extreme stress well
- **60-79**: ⚠️ Good - Some degradation under stress
- **40-59**: ⚠️ Poor - Moderate issues under stress
- **0-39**: ❌ Critical - Significant system problems

### Common Failure Modes
1. **Network Saturation**: High latency, packet loss >20%
2. **CAN Bus Overload**: Bus availability <80%, error rate >15%
3. **Command Conflicts**: Success rate <80%, high conflict counts
4. **Resource Exhaustion**: CPU >90%, memory leaks
5. **Cross-System Interference**: Conflicting priorities between systems

## Troubleshooting

### Network Test Issues
```bash
# Check if tc commands work
sudo tc qdisc show dev lo

# Reset network rules if stuck
sudo tc qdisc del dev lo root
```

### ROS2 Issues
```bash
# Check ROS2 installation
ros2 --version

# Verify DDS configuration
ros2 doctor
```

### Permission Issues
```bash
# Allow tc commands without sudo for testing
sudo setcap cap_net_admin+ep $(which python3)
```

## Generated Reports

The orchestrator generates two types of reports:

### JSON Results File
- **Location**: `stress_test_results/stress_test_results_YYYYMMDD_HHMMSS.json`
- **Contents**: Complete raw test data, metrics, and analysis
- **Use**: Detailed analysis, historical comparison, debugging

### Summary Report
- **Location**: `stress_test_results/stress_test_summary_YYYYMMDD_HHMMSS.txt`
- **Contents**: Human-readable summary, key findings, recommendations
- **Use**: Executive summary, quick assessment, documentation

## Example Output

```
🚀 URC 2026 Complete Communication Stress Test Suite
=============================================================

📡 PHASE 1: Network Communication Stress Tests
--------------------------------------------------
🔥 Testing MODERATE network conditions...
  Testing moderate network conditions for 15.0s...
  [5.0s] Sent: 750, Received: 735, Latency: 45.2ms, Dropped: 15
  ✅ Moderate network stress tests completed

📊 INTEGRATED SYSTEM STRESS TEST RESULTS
==================================================

🔗 INTRA-PROCESS:
  Messages sent: 1000
  Messages received: 1000
  Success rate: 100.0%
  Avg latency: 2.3ms
  P95 latency: 4.1ms

🌐 INTER-PROCESS:
  Messages sent: 1000
  Messages received: 950
  Success rate: 95.0%
  Avg latency: 45.2ms
  P95 latency: 78.9ms

⚡ PERFORMANCE IMPROVEMENTS:
  Latency reduction: 94.9% ⚡
  Throughput improvement: 5.3%
  CPU savings: 12.3%
  Memory savings: 0.00MB

🎯 URC 2026 SYSTEM IMPACT:
  • /imu (100Hz): State estimation accuracy
  • /cmd_vel (50Hz): Rover control responsiveness
  • /odom (50Hz): Better localization accuracy
  • vision/detections (30Hz): Improved obstacle avoidance
  • Safety topics: Critical safety response time reduction
```

## Integration with CI/CD

Add to your CI/CD pipeline:

```yaml
# .github/workflows/stress-tests.yml
name: Communication Stress Tests
on: [push, pull_request]

jobs:
  stress-test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - name: Setup ROS2
        uses: ros-tooling/setup-ros@v1
        with:
          required-ros-distributions: humble
      - name: Install dependencies
        run: pip install psutil
      - name: Run stress tests
        run: python3 tests/performance/stress_test_orchestrator.py --quick-test
      - name: Upload results
        uses: actions/upload-artifact@v2
        with:
          name: stress-test-results
          path: stress_test_results/
```

## Contributing

When adding new stress tests:

1. Follow the existing naming convention: `stress_test_[component].py`
2. Include comprehensive error handling
3. Provide clear performance metrics
4. Document test parameters and expected results
5. Add the test to the orchestrator if appropriate

## Safety Notes

⚠️ **Warning**: These tests intentionally stress communication systems beyond normal operating conditions. Some tests may cause:

- Temporary network connectivity issues
- High CPU/memory usage
- System responsiveness degradation
- Temporary loss of control interfaces

**Always run stress tests in isolated environments and have recovery procedures ready.**

## Support

For issues or questions:
1. Check the troubleshooting section above
2. Review generated log files in `stress_test_results/`
3. Examine individual test outputs for specific component failures
4. Consider environmental factors (system load, network conditions)

