.. _safety_procedures:

Emergency Procedures Manual
===========================

This manual provides comprehensive emergency procedures for the URC 2026 Mars Rover safety system. All team members must be familiar with these procedures before operating the rover.

.. warning::
   **CRITICAL**: Emergency procedures must be practiced regularly. Incorrect execution can result in equipment damage or safety hazards.

Emergency Stop Hierarchy
========================

The safety system implements a three-level emergency stop hierarchy:

Level 1: Software E-Stop
------------------------

**Trigger Conditions**:
- Safety violation detection (battery critical, thermal warning, sensor failure)
- State machine timeout or inconsistency
- Subsystem communication failure
- Manual software emergency stop

**Automatic Response**:
1. Safety violation logged with timestamp and context
2. Emergency stop signal sent to all subsystems
3. All autonomous operations halted immediately
4. LED status changes to red fast blink
5. Safety state machine transitions to EMERGENCY

**Response Time**: < 500ms from detection to full stop

**Recovery**: Manual intervention required (see Recovery Procedures)

Level 2: Hardware E-Stop
------------------------

**Trigger Conditions**:
- Physical emergency stop button pressed
- Critical hardware fault detected
- Power system anomaly

**Automatic Response**:
1. Hardware relay activation for immediate power cutoff to actuators
2. Software emergency stop triggered simultaneously
3. All motor power disconnected
4. Emergency state broadcast to all systems

**Response Time**: < 100ms from button press to power cutoff

**Recovery**: Hardware reset + manual intervention required

Level 3: Power E-Stop
--------------------

**Trigger Conditions**:
- Competition emergency declaration
- Complete system failure
- External safety intervention

**Manual Response**:
1. Main power disconnect activated
2. All systems powered down immediately
3. Emergency shutdown logged

**Response Time**: Immediate (power disconnection)

**Recovery**: Complete system restart required

Emergency Detection and Response
================================

Automatic Emergency Detection
-----------------------------

The safety system continuously monitors for emergency conditions:

**Battery Emergency**
   - **Critical Threshold**: < 10% battery remaining
   - **Detection**: Continuous monitoring via battery sensor
   - **Response**: Immediate transition to Level 1 E-Stop
   - **Recovery**: Battery replacement/charging required

**Thermal Emergency**
   - **Critical Threshold**: > 85°C system temperature
   - **Detection**: Temperature sensor monitoring
   - **Response**: Immediate cooling system activation + Level 1 E-Stop
   - **Recovery**: System cooling and temperature stabilization

**Sensor Failure Emergency**
   - **Detection**: Sensor data timeout or integrity failure
   - **Response**: Degraded mode operation or Level 1 E-Stop
   - **Recovery**: Sensor replacement or system restart

**Communication Emergency**
   - **Detection**: Loss of critical communication links
   - **Response**: Independent operation with increased safety margins
   - **Recovery**: Communication restoration

**Motion Safety Emergency**
   - **Detection**: Velocity limits exceeded or collision imminent
   - **Response**: Immediate motion halt
   - **Recovery**: Position verification and manual control

Manual Emergency Activation
---------------------------

**Software Emergency Stop**
   Command line activation::

   ros2 service call /state_machine/emergency_stop std_srvs/srv/Empty

   Or through safety dashboard emergency button.

**Hardware Emergency Stop**
   Physical button locations:
   - Primary: Rover main control panel
   - Secondary: Operator control station
   - Competition: URC-provided emergency stop button

**Remote Emergency Stop**
   Available through:
   - Web interface emergency stop button
   - ROS2 service calls from remote systems
   - Competition control system integration

Emergency Response Procedures
============================

Phase 1: Emergency Detection
----------------------------

**Immediate Actions** (0-5 seconds):

1. **Alert Acknowledgment**
   - Acknowledge emergency alert (audible/visual indicators)
   - Note emergency type and timestamp
   - Record operator location and activity

2. **System Status Assessment**
   - Check safety dashboard for emergency details
   - Verify rover physical status (position, orientation)
   - Confirm team member safety and location

3. **Communication**
   - Inform all team members of emergency
   - Establish emergency communication channel
   - Notify competition officials if applicable

**Commands**::

   # Check emergency status
   ros2 topic echo /safety/emergency_status --once

   # Check safety dashboard
   ros2 topic echo /safety/dashboard_status --once

Phase 2: Emergency Response
---------------------------

**System Halt Verification** (5-30 seconds):

1. **Verify Emergency Stop**
   - Confirm all motors stopped (visual inspection)
   - Check LED status (should be red fast blink)
   - Verify autonomous operations halted

2. **Subsystem Status Check**
   - Check all subsystem emergency acknowledgments
   - Verify power systems stable
   - Confirm communication links

3. **Environmental Assessment**
   - Assess rover stability and position
   - Check for obstacles or hazards
   - Evaluate weather/terrain conditions

**Commands**::

   # Check subsystem emergency responses
   ros2 topic echo /safety/emergency_coordination --once

   # Verify system state
   ros2 service call /state_machine/get_system_state autonomy_interfaces/srv/GetSystemState

Phase 3: Emergency Containment
------------------------------

**Hazard Mitigation** (30 seconds - 2 minutes):

1. **Power Management**
   - Assess battery status and power requirements
   - Implement power conservation if needed
   - Prepare for extended emergency state

2. **System Stabilization**
   - Prevent system state changes during emergency
   - Log all system parameters for analysis
   - Prepare diagnostic information

3. **Team Coordination**
   - Assign emergency response roles
   - Establish incident command structure
   - Document all actions taken

Phase 4: Recovery Assessment
----------------------------

**System Evaluation** (2-5 minutes):

1. **Root Cause Analysis**
   - Determine emergency trigger source
   - Assess system damage or degradation
   - Evaluate recovery feasibility

2. **Recovery Planning**
   - Select appropriate recovery procedure
   - Prepare recovery checklist
   - Brief team on recovery plan

3. **Safety Verification**
   - Confirm emergency containment
   - Verify system stability for recovery
   - Check environmental conditions

Recovery Procedures
==================

Automatic Recovery
------------------

**Conditions for Automatic Recovery**:
- Emergency was Level 1 (Software E-Stop)
- Root cause identified and resolved
- System health checks pass
- No hardware damage detected

**Procedure**::

   # Request automatic recovery
   ros2 service call /state_machine/recover_from_safety autonomy_interfaces/srv/RecoverFromSafety "{
     recovery_method: 'AUTO',
     operator_id: 'recovery_operator',
     acknowledge_risks: true,
     notes: 'Battery replaced, system cooled'
   }"

   # Monitor recovery progress
   ros2 topic echo /safety/recovery_coordination

   # Verify recovery completion
   ros2 service call /state_machine/get_system_state autonomy_interfaces/srv/GetSystemState

Manual Guided Recovery
----------------------

**Conditions for Manual Recovery**:
- Emergency requires specific manual intervention
- System requires step-by-step validation
- Operator needs to verify each recovery step

**Procedure**::

   # Request manual guided recovery
   ros2 service call /state_machine/recover_from_safety autonomy_interfaces/srv/RecoverFromSafety "{
     recovery_method: 'MANUAL_GUIDED',
     operator_id: 'recovery_operator',
     acknowledge_risks: true,
     completed_steps: ['Battery replaced', 'Temperature normalized'],
     notes: 'Manual intervention completed'
   }"

   # Transition to teleoperation for manual control
   ros2 service call /state_machine/change_state autonomy_interfaces/srv/ChangeState "{
     desired_state: 'TELEOPERATION',
     reason: 'Manual recovery from emergency',
     operator_id: 'recovery_operator'
   }"

Full System Reset
-----------------

**Conditions for Full Reset**:
- Emergency caused system instability
- Multiple subsystem failures
- Configuration corruption suspected
- As last resort when other recovery fails

**Procedure**::

   # Full system reset (use only as last resort)
   ros2 service call /state_machine/recover_from_safety autonomy_interfaces/srv/RecoverFromSafety "{
     recovery_method: 'FULL_RESET',
     operator_id: 'recovery_operator',
     acknowledge_risks: true,
     notes: 'Complete system reset required'
   }"

   # System will restart boot sequence
   ros2 topic echo /state_machine/current_state

   # Wait for boot completion
   ros2 topic echo /state_machine/subsystem_status

Competition Emergency Procedures
================================

URC 2026 Competition Requirements
---------------------------------

**Emergency Stop Time**: Must respond within 3 seconds of detection

**Emergency Stop Accessibility**:
- Must be accessible within 3 meters of rover
- Must be clearly marked and illuminated
- Must be operable with one hand
- Must remain operable after activation

**Competition Emergency Declaration**:
- Competition officials may declare competition emergency
- All teams must activate emergency stops immediately
- No operation allowed until officials clear emergency

**Post-Emergency Procedures**:
1. Remain at emergency location
2. Do not touch rover until cleared by officials
3. Prepare incident report for judges
4. Await technical inspection before resuming

Competition Emergency Response
------------------------------

**During Competition Run**:

1. **Immediate Stop**
   - Activate emergency stop immediately
   - Do not move rover or touch controls
   - Wait for official clearance

2. **Emergency Assessment**
   - Note time and location of emergency
   - Observe but do not interfere with rover
   - Prepare to answer judge questions

3. **Post-Emergency**
   - Follow judge instructions exactly
   - Do not resume operation without clearance
   - Document emergency for competition report

**Commands During Competition**::

   # Emergency stop (if software accessible)
   ros2 service call /state_machine/emergency_stop std_srvs/srv/Empty

   # Check competition status (if available)
   ros2 topic echo /competition/status

Testing and Validation Procedures
=================================

Emergency System Testing
------------------------

**Daily Safety Checks**:

1. **Hardware Emergency Stop Test**
   - Press emergency stop button
   - Verify immediate response
   - Reset and verify normal operation

2. **Software Emergency Stop Test**
   - Trigger software emergency stop
   - Verify system halt
   - Test recovery procedures

3. **Safety Threshold Testing**
   - Test battery low warnings
   - Test thermal warnings
   - Test sensor failure detection

**Weekly Safety Validation**:

1. **Full Emergency Scenario Testing**
   - Complete emergency stop and recovery cycle
   - Test under various conditions
   - Validate timing requirements

2. **Multi-System Failure Testing**
   - Test response to multiple simultaneous failures
   - Validate safety system redundancy
   - Test degraded mode operation

**Monthly Safety Audits**:

1. **Safety System Performance Review**
   - Analyze emergency response times
   - Review false positive/negative rates
   - Update safety thresholds as needed

2. **Team Emergency Procedure Drills**
   - Practice emergency response procedures
   - Test communication during emergencies
   - Validate team coordination

Automated Safety Testing
------------------------

**Continuous Safety Monitoring**::

   # Run automated safety tests
   ros2 launch autonomy_safety_system safety_system.launch.py enable_integration_tester:=true

   # Check test results
   ros2 topic echo /safety/test_status

**Safety Regression Testing**::

   # Run full safety test suite
   ros2 run autonomy_safety_system safety_integration_tester

   # Review test results
   ros2 topic echo /safety/test_status --filter "completed_tests > 0"

Emergency Procedure Checklist
============================

Pre-Operation Emergency Checklist
---------------------------------

**Daily Pre-Operation**:
- [ ] Emergency stop buttons tested and functional
- [ ] Safety system status checked (green/ready)
- [ ] Battery level verified (> 20%)
- [ ] Temperature within safe limits
- [ ] Communication links verified
- [ ] Team emergency procedures reviewed

**Competition Pre-Run**:
- [ ] URC emergency stop procedures confirmed
- [ ] Competition emergency zones identified
- [ ] Emergency communication channels established
- [ ] Team emergency roles assigned
- [ ] Emergency equipment prepared

During Operation Emergency Checklist
------------------------------------

**Emergency Detection**:
- [ ] Emergency acknowledged immediately
- [ ] Emergency type identified and logged
- [ ] Team alerted and communication established
- [ ] Safety dashboard checked for details

**Emergency Response**:
- [ ] Emergency stop activated (appropriate level)
- [ ] System halt verified (motors stopped, LEDs red)
- [ ] Subsystem responses confirmed
- [ ] Environmental hazards assessed

**Emergency Containment**:
- [ ] Power management implemented
- [ ] System stabilization achieved
- [ ] Incident documentation started
- [ ] Team coordination established

**Recovery Assessment**:
- [ ] Root cause identified
- [ ] Recovery method selected
- [ ] Recovery checklist prepared
- [ ] Safety verification completed

Post-Emergency Checklist
------------------------

**Immediate Post-Emergency**:
- [ ] Emergency contained and safe
- [ ] Team member safety confirmed
- [ ] Equipment damage assessed
- [ ] Incident report initiated

**Recovery Execution**:
- [ ] Recovery procedure followed correctly
- [ ] System validation completed
- [ ] Normal operation restored
- [ ] Incident report completed

**Follow-up Actions**:
- [ ] Root cause analysis completed
- [ ] System improvements identified
- [ ] Team debrief conducted
- [ ] Documentation updated

Emergency Communication Protocols
================================

Internal Team Communication
---------------------------

**Emergency Alert**:
- **Phrase**: "EMERGENCY STOP - [Type] - [Location]"
- **Response**: All team members acknowledge immediately
- **Channel**: Dedicated emergency radio channel or verbal

**Status Updates**:
- **Format**: "[Time] [Location] [Status] [Next Action]"
- **Frequency**: Every 30 seconds during active emergency
- **Channel**: Emergency communication channel

**Recovery Coordination**:
- **Format**: "[Recovery Phase] [Status] [Blockers]"
- **Frequency**: Every 1 minute during recovery
- **Channel**: Recovery coordination channel

External Communication
----------------------

**Competition Officials**:
- **During Competition**: Immediate emergency declaration
- **Format**: "Team [Number] Emergency Stop - [Reason] - [Location]"
- **Response**: Await official instructions

**Medical Emergency**:
- **Priority**: Highest priority over all other procedures
- **Response**: Activate local emergency services immediately
- **Notification**: Competition officials and team leadership

**Equipment Damage**:
- **Assessment**: Document all damage for insurance/reporting
- **Reporting**: Notify competition officials and team leadership
- **Containment**: Secure damaged equipment to prevent further issues

Emergency Documentation Requirements
====================================

Incident Report Format
----------------------

**Required Information**:
1. **Incident Details**
   - Date and time of incident
   - Location (GPS coordinates if available)
   - Team members present
   - Operating mode (teleop/autonomous/testing)

2. **Emergency Trigger**
   - Type of emergency (software/hardware/power)
   - Trigger source (battery/sensor/communication/manual)
   - Safety system response details
   - False positive/negative assessment

3. **Response Actions**
   - Emergency stop level activated
   - Response time measurements
   - Manual actions taken
   - Team coordination details

4. **Recovery Process**
   - Recovery method used (auto/manual/reset)
   - Time to recovery completion
   - System validation results
   - Lessons learned

5. **Follow-up Actions**
   - Root cause analysis results
   - System improvements implemented
   - Training updates required
   - Competition impact assessment

**Submission Requirements**:
- Incident report completed within 24 hours
- Submitted to team leadership and competition officials
- Included in competition technical report
- Used for safety system improvement

Safety System Maintenance
=========================

Regular Maintenance Procedures
------------------------------

**Daily Maintenance**:
- Emergency stop button functionality check
- Safety system status verification
- Battery and temperature sensor calibration check
- Safety log review for anomalies

**Weekly Maintenance**:
- Complete safety system test cycle
- Emergency stop response time measurement
- Safety threshold verification
- Team emergency procedure drill

**Monthly Maintenance**:
- Safety system performance analysis
- Emergency response time trending
- False positive/negative rate analysis
- Safety system software updates

**Competition Maintenance**:
- Pre-competition safety system validation
- Emergency stop accessibility verification
- Competition rule compliance check
- Team emergency procedure certification

Safety System Updates
---------------------

**Software Updates**:
- Safety system software updates tested in development
- Backward compatibility verified
- Emergency procedures updated for changes
- Team training conducted for new features

**Configuration Updates**:
- Safety thresholds validated before deployment
- Configuration changes documented
- Rollback procedures prepared
- System validation after changes

**Hardware Updates**:
- Emergency stop button functionality verified
- Power system integration tested
- Sensor calibration updated
- System integration testing completed

This emergency procedures manual provides comprehensive guidance for handling all emergency situations with the URC 2026 Mars Rover. Regular training and validation of these procedures is essential for safe and successful competition participation.
