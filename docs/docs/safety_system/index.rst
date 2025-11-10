.. _safety_system:

Safety System Documentation
===========================

The URC 2026 Safety System provides comprehensive protection for the Mars rover through multi-layered monitoring, emergency response coordination, and automated safety validation. This system ensures safe operation during both development and competition phases.

.. warning::
   **Competition Safety Notice**: This safety system is designed to meet URC 2026 competition requirements. All safety features must be validated before competition use.

.. toctree::
   :maxdepth: 2
   :caption: Safety System Documentation

   architecture
   components
   procedures
   testing
   integration
   configuration

System Overview
===============

The safety system implements a **defense-in-depth** approach with multiple independent safety layers:

**Layer 1: Primary Safety System**
   - State machine safety monitoring
   - Subsystem health tracking
   - Basic emergency stop functionality

**Layer 2: Independent Watchdog System**
   - Hardware-independent safety monitoring
   - Multi-level violation detection
   - Emergency stop coordination

**Layer 3: Redundant Safety Monitor**
   - Independent validation of primary systems
   - Sensor data integrity checking
   - Cross-system consistency verification

**Layer 4: Emergency Response Coordinator**
   - Coordinated emergency stop execution
   - Multi-subsystem response management
   - Automated recovery procedures

**Layer 5: Safety Dashboard & Testing**
   - Real-time safety visualization
   - Automated safety scenario testing
   - System validation and monitoring

Key Features
============

🔒 **Multi-Layered Protection**
   Independent safety systems prevent single-point failures

🚨 **Real-Time Monitoring**
   Continuous health monitoring with immediate violation detection

⚡ **Rapid Emergency Response**
   Coordinated emergency stops across all subsystems within 500ms

🔄 **Automated Recovery**
   Intelligent recovery procedures with safety validation

📊 **Comprehensive Diagnostics**
   Real-time safety status and detailed diagnostic information

🧪 **Automated Testing**
   Complete safety validation through automated test scenarios

Architecture Benefits
=====================

**Competition Ready**
   - Meets URC 2026 safety requirements
   - Validated emergency stop procedures
   - Competition-specific safety thresholds

**Development Friendly**
   - Software-only implementation for early testing
   - Comprehensive logging and diagnostics
   - Automated test scenarios for validation

**Maintainable**
   - Modular design with clear interfaces
   - Extensive documentation and procedures
   - Automated testing for regression prevention

Quick Start
===========

1. **Launch Safety System**::

   ros2 launch autonomy_safety_system safety_system.launch.py

2. **Monitor Safety Status**::

   ros2 topic echo /safety/dashboard_status

3. **Run Safety Tests**::

   ros2 launch autonomy_safety_system safety_system.launch.py enable_integration_tester:=true

4. **Check Emergency Procedures**::

   ros2 topic echo /safety/emergency_status

Safety States
=============

The safety system operates in these primary states:

**NORMAL**
   All systems healthy, normal operation allowed

**MONITORING**
   Active monitoring, potential issues detected but operation continues

**WARNING**
   Warning conditions detected, prepare for potential intervention

**CRITICAL**
   Critical conditions, immediate attention required

**EMERGENCY**
   Emergency halt required, all operations stopped

Safety Violations
================

The system monitors for these violation types:

**Battery Safety**
   - Critical battery level (< 10%)
   - Battery drain rate anomalies
   - Power system failures

**Thermal Safety**
   - Over-temperature conditions (> 85°C)
   - Thermal runaway detection
   - Cooling system failures

**Sensor Safety**
   - Sensor data integrity failures
   - Stuck sensor detection
   - Communication timeouts

**Motion Safety**
   - Velocity limit violations
   - Obstacle detection failures
   - Position uncertainty issues

**System Safety**
   - Subsystem communication failures
   - State machine timeouts
   - Watchdog system failures

Emergency Response Hierarchy
===========================

**Level 1: Software E-Stop**
   - Triggered by software safety violations
   - Coordinated shutdown of all subsystems
   - Recovery through operator intervention

**Level 2: Hardware E-Stop**
   - Physical emergency stop button
   - Immediate power cutoff to actuators
   - Requires manual reset

**Level 3: Power E-Stop**
   - Complete power disconnection
   - Hardware-based power cutoff
   - Competition emergency procedure

System Components
=================

**Safety Watchdog** (`safety_watchdog`)
   Independent monitoring system that validates system health

**Redundant Safety Monitor** (`redundant_safety_monitor`)
   Secondary safety validation system for redundancy

**Emergency Response Coordinator** (`emergency_response_coordinator`)
   Coordinates emergency responses across all subsystems

**Safety Dashboard** (`safety_dashboard`)
   Real-time safety monitoring and visualization

**Safety Integration Tester** (`safety_integration_tester`)
   Automated testing framework for safety validation

Monitoring Topics
=================

**Core Safety Topics**
   - ``/safety/dashboard_status`` - Overall safety system status
   - ``/safety/active_alerts`` - Currently active safety alerts
   - ``/safety/emergency_status`` - Emergency response coordination
   - ``/safety/system_health`` - Individual system health status

**Diagnostic Topics**
   - ``/safety/diagnostics`` - ROS2 diagnostic information
   - ``/safety/redundant_diagnostics`` - Redundant system diagnostics
   - ``/safety/emergency_diagnostics`` - Emergency response diagnostics

**Testing Topics**
   - ``/safety/test_status`` - Safety test execution status
   - ``/safety/test_control`` - Safety test control interface

Configuration
=============

The safety system is configured through YAML files:

**Main Configuration** (`config/safety_system.yaml`)
   Complete safety system configuration with thresholds and settings

**Launch Parameters**
   Runtime configuration through launch file parameters

**ROS2 Parameters**
   Dynamic reconfiguration through ROS2 parameter server

Competition Requirements
========================

**URC 2026 Compliance**
   - Emergency stop within 3 seconds of detection
   - Independent safety monitoring systems
   - Operator emergency stop accessibility
   - Safety system validation documentation

**Validation Requirements**
   - Safety system testing procedures
   - Emergency response time validation
   - Safety threshold verification
   - Recovery procedure validation

**Documentation Requirements**
   - Safety system architecture documentation
   - Emergency procedure documentation
   - Safety testing validation reports
   - System maintenance procedures

Next Steps
==========

1. **Review Architecture** - Understand the multi-layered safety approach
2. **Configure System** - Set appropriate safety thresholds for your environment
3. **Test Integration** - Run automated safety tests to validate system operation
4. **Train Procedures** - Review and practice emergency response procedures
5. **Validate Competition Ready** - Ensure compliance with URC 2026 requirements
