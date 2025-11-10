.. _safety_architecture:

Safety System Architecture
==========================

The URC 2026 Safety System implements a comprehensive, multi-layered safety architecture designed to protect the Mars rover during all phases of operation. This document details the architectural design, component interactions, and safety principles.

Architectural Principles
========================

Defense in Depth
----------------

The safety system follows a **defense-in-depth** strategy with multiple independent safety layers:

.. figure:: ../_static/safety_architecture_layers.png
   :alt: Safety System Layers
   :align: center
   :scale: 75%

   Multi-layered safety architecture with independent protection levels

**Layer 1: Application Safety**
   - State machine safety monitoring
   - Subsystem health validation
   - Application-level safety checks

**Layer 2: System Safety**
   - Independent watchdog monitoring
   - Hardware-independent safety validation
   - System-level health assessment

**Layer 3: Redundant Safety**
   - Secondary safety state machine
   - Cross-system consistency checking
   - Independent sensor validation

**Layer 4: Emergency Response**
   - Coordinated emergency stop execution
   - Multi-subsystem response management
   - Recovery procedure coordination

**Layer 5: Safety Assurance**
   - Real-time safety monitoring dashboard
   - Automated safety testing and validation
   - Continuous safety system assessment

Single Point of Failure Prevention
----------------------------------

The architecture eliminates single points of failure through:

**Independent Monitoring**
   Each safety layer operates independently with separate:
   - Power sources (when available)
   - Communication channels
   - Processing resources
   - Decision logic

**Redundant Validation**
   Safety decisions are validated by multiple independent systems:
   - Primary and redundant safety monitors
   - Cross-check validation between systems
   - Consensus-based safety decisions

**Graceful Degradation**
   System maintains maximum possible safety level even during failures:
   - Automatic fallback to higher safety levels
   - Continued operation with reduced capabilities
   - Clear indication of degraded safety status

Component Architecture
======================

Safety Watchdog System
-----------------------

**Purpose**: Independent safety monitoring and emergency stop coordination

**Architecture**:

.. code-block:: none

   +-------------------+     +-------------------+
   | Heartbeat Monitor |     | State Transition  |
   | - System pulse     |     | - State changes   |
   | - Timeout detection|     | - Transition time |
   +-------------------+     +-------------------+
           |                           |
           v                           v
   +-------------------+     +-------------------+
   | Subsystem Health  |     | Sensor Integrity  |
   | - Component status |     | - Data validation |
   | - Failure detection|     | - Quality checks  |
   +-------------------+     +-------------------+
                   |
                   v
   +-------------------+
   | Emergency Stop    |
   | Coordinator       |
   | - Violation        |
   |   assessment       |
   | - Response         |
   |   coordination     |
   +-------------------+

**Key Features**:
- Multi-level monitoring (Heartbeat, State, Subsystem, Sensor)
- Independent operation from main autonomy stack
- Emergency stop signal generation and coordination
- Comprehensive violation logging and reporting

Redundant Safety Monitor
-------------------------

**Purpose**: Secondary safety validation and cross-system consistency checking

**Architecture**:

.. code-block:: none

   +-------------------+     +-------------------+
   | Primary Safety    | --> | Consistency       |
   | System Monitor    |     | Check             |
   +-------------------+     +-------------------+
           |                           |
           v                           v
   +-------------------+     +-------------------+
   | Independent       |     | Sensor Data       |
   | Sensor Validation |     | Integrity Check   |
   +-------------------+     +-------------------+
                   |
                   v
   +-------------------+
   | Redundant Safety  |
   | Status & Alerts   |
   +-------------------+

**Key Features**:
- Independent sensor data validation
- Cross-system consistency verification
- Secondary safety state machine
- Failure mode detection and reporting

Emergency Response Coordinator
------------------------------

**Purpose**: Coordinated emergency response across all rover subsystems

**Architecture**:

.. code-block:: none

   +-------------------+
   | Emergency         |
   | Detection &       |
   | Assessment        |
   +-------------------+
           |
           v
   +-------------------+
   | Response          |
   | Coordination      |
   | - Parallel E-stop  |
   | - Timeout handling |
   | - Status tracking  |
   +-------------------+
           |
           v
   +-------------------+
   | Recovery          |
   | Management        |
   | - State recovery   |
   | - Validation       |
   | - Status reporting |
   +-------------------+

**Key Features**:
- Coordinated emergency stop execution
- Multi-subsystem response tracking
- Recovery procedure management
- Emergency event logging and analysis

Safety Dashboard
----------------

**Purpose**: Real-time safety monitoring, visualization, and alerting

**Architecture**:

.. code-block:: none

   +-------------------+     +-------------------+
   | Data Aggregation  |     | Alert Processing  |
   | - Multi-source     |     | - Violation       |
   |   collection      |     |   assessment      |
   | - Normalization   |     | - Escalation      |
   +-------------------+     +-------------------+
           |                           |
           v                           v
   +-------------------+     +-------------------+
   | Health Assessment |     | System Status     |
   | - Component health |     | - Overall status  |
   | - Trend analysis  |     | - Alert summary    |
   +-------------------+     +-------------------+
                   |
                   v
   +-------------------+
   | Visualization &   |
   | Reporting         |
   | - Real-time        |
   |   dashboard        |
   | - Diagnostic       |
   |   reporting        |
   +-------------------+

**Key Features**:
- Real-time safety status aggregation
- Alert generation and management
- System health assessment and trending
- Diagnostic reporting and analysis

Safety Integration Tester
-------------------------

**Purpose**: Automated safety system validation and testing

**Architecture**:

.. code-block:: none

   +-------------------+
   | Test Scenario     |
   | Definition        |
   | - Battery critical |
   | - Sensor failure   |
   | - Emergency stop   |
   +-------------------+
           |
           v
   +-------------------+
   | Test Execution    |
   | - Simulation       |
   | - Monitoring       |
   | - Validation       |
   +-------------------+
           |
           v
   +-------------------+
   | Result Analysis   |
   | - Success/failure  |
   | - Performance      |
   | - Recommendations  |
   +-------------------+

**Key Features**:
- Automated safety scenario testing
- Performance validation under load
- System response validation
- Test result analysis and reporting

Data Flow Architecture
======================

Safety Data Flow
----------------

The safety system processes data through multiple validation layers:

.. code-block:: none

   Sensor Data --> Primary Validation --> Safety Assessment --> Emergency Response
       |               |                        |                      |
       v               v                        v                      v
   Redundant --> Cross-Check --> Alert Generation --> Coordinated --> Recovery
   Validation   Validation    Escalation         Action          Procedures

**Data Sources**:
- ROS2 sensor topics (IMU, GPS, battery, temperature)
- System state topics (state machine, subsystem status)
- Diagnostic topics (component health, error conditions)
- Heartbeat topics (system pulse monitoring)

**Data Processing**:
- Real-time validation against safety thresholds
- Statistical analysis for anomaly detection
- Cross-correlation between multiple data sources
- Historical trending for predictive safety assessment

Communication Architecture
==========================

ROS2 Topic-Based Communication
------------------------------

The safety system uses ROS2 topics for all inter-component communication:

**Safety Monitoring Topics**:
- ``/safety/dashboard_status`` - Overall system safety status
- ``/safety/active_alerts`` - Currently active safety alerts
- ``/safety/system_health`` - Individual component health
- ``/safety/sensor_health`` - Sensor data quality assessment

**Emergency Response Topics**:
- ``/safety/emergency_stop`` - Emergency stop signal coordination
- ``/safety/emergency_status`` - Emergency response coordination status
- ``/safety/recovery_coordination`` - Recovery procedure coordination

**Diagnostic Topics**:
- ``/safety/diagnostics`` - ROS2 diagnostic information
- ``/safety/redundant_diagnostics`` - Redundant system diagnostics
- ``/safety/emergency_diagnostics`` - Emergency response diagnostics

**Testing Topics**:
- ``/safety/test_status`` - Safety test execution status
- ``/safety/test_control`` - Safety test control interface

QoS Configuration
-----------------

Safety-critical communications use appropriate QoS settings:

**Real-time Safety Data** (IMU, emergency signals):
- Reliability: BEST_EFFORT (minimize latency)
- Durability: VOLATILE (only current data matters)
- History: KEEP_LAST, depth=10

**Status and Diagnostics** (health monitoring):
- Reliability: RELIABLE (ensure delivery)
- Durability: TRANSIENT_LOCAL (late joiners get data)
- History: KEEP_LAST, depth=5

**Alert Communications** (safety violations):
- Reliability: RELIABLE (critical alerts must be delivered)
- Durability: TRANSIENT_LOCAL (ensure visibility)
- History: KEEP_LAST, depth=20

Safety State Machine
===================

The safety system implements a hierarchical safety state machine:

.. code-block:: none

   +-----------+     +-----------+     +-----------+
   |   NORMAL  | --> |  WARNING  | --> |  CRITICAL |
   +-----------+     +-----------+     +-----------+
         ^                 |                 |
         |                 v                 v
   +-----------+     +-----------+     +-----------+
   | RECOVERY  | <-- | MONITORING| <-- | EMERGENCY |
   +-----------+     +-----------+     +-----------+

**State Definitions**:

**NORMAL**
   - All systems healthy
   - Normal operation allowed
   - Standard monitoring active

**MONITORING**
   - Active monitoring of potential issues
   - Operation continues but with increased scrutiny
   - Early warning system active

**WARNING**
   - Warning conditions detected
   - Prepare for potential intervention
   - Increased monitoring frequency

**CRITICAL**
   - Critical conditions detected
   - Immediate attention required
   - Operation continues but with restrictions

**EMERGENCY**
   - Emergency halt required
   - All operations stopped
   - Emergency response procedures activated

**RECOVERY**
   - Recovery from emergency state
   - System validation and testing
   - Gradual return to normal operation

State Transition Logic
----------------------

**Automatic Transitions**:
- NORMAL → MONITORING: Minor issues detected
- MONITORING → WARNING: Issues persist or worsen
- WARNING → CRITICAL: Critical thresholds exceeded
- CRITICAL → EMERGENCY: Emergency conditions detected

**Manual Transitions**:
- Any State → EMERGENCY: Emergency stop activated
- EMERGENCY → RECOVERY: Recovery procedures initiated
- RECOVERY → NORMAL: System validation successful

**Recovery Transitions**:
- EMERGENCY → RECOVERY: Operator initiates recovery
- RECOVERY → NORMAL: All safety checks pass
- RECOVERY → EMERGENCY: Recovery fails, re-enter emergency

Failure Mode Analysis
====================

Single Component Failures
--------------------------

**Safety Watchdog Failure**:
- Detection: Redundant safety monitor detects missing watchdog signals
- Response: Automatic escalation to higher safety level
- Recovery: Restart watchdog system, validate operation

**Redundant Monitor Failure**:
- Detection: Safety dashboard detects communication loss
- Response: Increased monitoring of primary systems
- Recovery: Restart redundant monitor, synchronize state

**Emergency Coordinator Failure**:
- Detection: Watchdog detects lack of emergency response coordination
- Response: Fallback to individual subsystem emergency stops
- Recovery: Restart coordinator, validate coordination capability

**Safety Dashboard Failure**:
- Detection: Watchdog monitors dashboard heartbeat
- Response: Continue operation with reduced monitoring
- Recovery: Restart dashboard, restore monitoring capability

Multiple Component Failures
----------------------------

**Dual Safety System Failure**:
- Detection: Hardware watchdog or manual monitoring
- Response: Immediate transition to fail-safe mode
- Recovery: Complete system restart and validation

**Communication System Failure**:
- Detection: Loss of ROS2 communication between components
- Response: Independent operation with local safety decisions
- Recovery: Restore communication, synchronize safety state

**Power System Failures**:
- Detection: Battery monitoring and independent power sensors
- Response: Graceful power-down procedures
- Recovery: External power source connection and validation

Safety Validation Architecture
==============================

Automated Testing Framework
---------------------------

The safety system includes comprehensive automated testing:

**Unit Testing**:
- Individual component safety logic validation
- Threshold checking and response validation
- Communication protocol verification

**Integration Testing**:
- Multi-component safety system validation
- Cross-system communication verification
- End-to-end emergency response testing

**System Testing**:
- Full rover safety system validation
- Performance testing under load
- Failure mode injection and recovery testing

**Competition Testing**:
- URC 2026 specific safety requirement validation
- Emergency stop time measurement and verification
- Safety system endurance testing

Performance Requirements
========================

Timing Requirements
-------------------

**Emergency Stop Response**:
- Detection to action: < 100ms
- Complete system halt: < 500ms
- URC 2026 requirement: < 3 seconds

**Safety Monitoring**:
- Sensor data validation: < 50ms
- Safety assessment: < 100ms
- Alert generation: < 200ms

**System Health Checks**:
- Component health: < 1 second intervals
- System-wide assessment: < 5 seconds
- Dashboard updates: < 1 second intervals

Resource Requirements
---------------------

**CPU Usage**:
- Normal operation: < 5% total CPU
- Active monitoring: < 15% total CPU
- Emergency response: < 25% total CPU

**Memory Usage**:
- Base system: < 50MB RAM
- With monitoring: < 100MB RAM
- During testing: < 200MB RAM

**Network Usage**:
- Normal operation: < 1Mbps
- Emergency response: < 5Mbps
- Testing: < 10Mbps

**Storage Requirements**:
- Log files: < 100MB/day
- Test results: < 500MB/month
- Configuration: < 1MB

Reliability Requirements
========================

**Availability**:
- Safety system uptime: > 99.9%
- Mean time between failures: > 100 hours
- Mean time to recovery: < 30 seconds

**Fault Detection**:
- Safety violation detection: > 95% coverage
- False positive rate: < 1%
- Detection latency: < 200ms

**Recovery**:
- Automatic recovery success rate: > 90%
- Manual recovery time: < 5 minutes
- Data integrity during recovery: 100%

**Testing Coverage**:
- Safety scenario test coverage: > 95%
- Regression test execution: Daily
- Performance validation: Continuous

This architecture provides comprehensive safety protection while maintaining system performance and reliability requirements for the URC 2026 Mars rover competition.
