===================
Communication Patterns
===================

This document illustrates the communication patterns and data flows within the URC 2026 system using automated diagram generation.

System Communication Overview
=============================

.. uml::

   @startuml System Communication Overview
   !theme plain
   skinparam backgroundColor #FEFEFE
   skinparam sequenceParticipant underline

   actor Operator
   participant "Web Frontend" as Frontend
   participant "ROS2 Bridge" as ROSBridge
   participant "State Machine" as StateMachine
   participant "Navigation" as Navigation
   participant "Computer Vision" as Vision
   participant "Hardware" as Hardware

   Operator -> Frontend: Mission command
   Frontend -> ROSBridge: WebSocket message
   ROSBridge -> StateMachine: ROS2 service call
   StateMachine -> StateMachine: Validate transition
   StateMachine -> Navigation: Execute navigation
   Navigation -> Vision: Request obstacle detection
   Vision -> Navigation: Return obstacle map
   Navigation -> Hardware: Send motor commands
   Hardware -> Navigation: Odometry feedback
   Navigation -> StateMachine: Progress update
   StateMachine -> ROSBridge: Status message
   ROSBridge -> Frontend: WebSocket update
   Frontend -> Operator: UI update
   @enduml

State Machine Transitions
========================

.. uml::

   @startuml State Machine
   [*] --> BOOT : System start
   BOOT --> CALIBRATION : boot_to_calibration
   BOOT --> IDLE : boot_to_idle
   BOOT --> SAFETY : boot_to_safety

   CALIBRATION --> IDLE : calibration_to_idle
   CALIBRATION --> SAFETY : calibration_to_safety

   IDLE --> CALIBRATION : idle_to_calibration
   IDLE --> TELEOPERATION : idle_to_teleop
   IDLE --> AUTONOMOUS : idle_to_autonomous

   TELEOPERATION --> IDLE : teleop_to_idle
   TELEOPERATION --> SAFETY : teleop_to_safety

   AUTONOMOUS --> IDLE : autonomous_to_idle
   AUTONOMOUS --> SAFETY : autonomous_to_safety

   SAFETY --> IDLE : safety_to_idle
   SAFETY --> SHUTDOWN : safety_to_shutdown

   SHUTDOWN --> [*] : System halt

   @enduml

Component Dependencies
======================

.. uml::

   @startuml Component Dependencies
   !theme plain
   skinparam componentStyle uml2

   package "User Interface" {
     [Web Frontend]
     [3D Visualization]
   }

   package "ROS2 Middleware" {
     [ROS2 Bridge]
     [Message Router]
   }

   package "Autonomy Stack" {
     [State Machine]
     [Navigation]
     [Computer Vision]
     [SLAM]
     [Calibration]
   }

   package "Hardware Interfaces" {
     [Motor Controllers]
     [Sensors]
     [Communication]
   }

   [Web Frontend] --> [ROS2 Bridge]
   [3D Visualization] --> [ROS2 Bridge]
   [ROS2 Bridge] --> [State Machine]
   [ROS2 Bridge] --> [Navigation]
   [ROS2 Bridge] --> [Computer Vision]
   [ROS2 Bridge] --> [SLAM]
   [ROS2 Bridge] --> [Calibration]
   [State Machine] --> [Navigation]
   [Navigation] --> [Computer Vision]
   [Navigation] --> [SLAM]
   [Computer Vision] --> [Sensors]
   [SLAM] --> [Sensors]
   [Calibration] --> [Sensors]
   [Navigation] --> [Motor Controllers]
   [State Machine] --> [Motor Controllers]
   [Motor Controllers] --> [Communication]
   [Sensors] --> [Communication]
   @enduml

Data Flow Architecture
======================

.. uml::

   @startuml Data Flow
   !theme plain

   rectangle "Data Sources" as sources {
     card "Sensors\n(Camera, IMU, GPS)" as sensors
     card "User Input\n(Web UI, Commands)" as user_input
     card "External Systems\n(Simulation, APIs)" as external
   }

   rectangle "Processing Pipeline" as processing {
     card "Perception\n(Computer Vision)" as perception
     card "Localization\n(SLAM)" as localization
     card "Planning\n(Navigation)" as planning
     card "Control\n(Motion Control)" as control
   }

   rectangle "Data Sinks" as sinks {
     card "Actuators\n(Motors, Servos)" as actuators
     card "User Feedback\n(Web UI, Logs)" as feedback
     card "External Systems" as external_out
   }

   sensors --> perception : Raw sensor data
   user_input --> planning : Mission goals
   external --> processing : External commands

   perception --> localization : Feature detections
   perception --> planning : Obstacle maps
   localization --> planning : Pose estimates
   planning --> control : Motion commands

   control --> actuators : Control signals
   control --> feedback : Status updates
   planning --> feedback : Progress reports

   feedback --> user_input : [Feedback loop]
   actuators --> sensors : [Sensor feedback]
   @enduml

ROS2 Topic Network
==================

.. uml::

   @startuml ROS2 Topics
   !theme plain
   skinparam nodesep 50
   skinparam ranksep 30

   node "Frontend" as frontend {
     portin "/mission/goal" as frontend_goal
     portout "/ui/status" as frontend_status
   }

   node "State Machine" as statemachine {
     portin "/mission/goal" as sm_goal
     portout "/state/current" as sm_current
     portout "/state/transitions" as sm_transitions
   }

   node "Navigation" as navigation {
     portin "/nav/goal" as nav_goal
     portout "/odom" as nav_odom
     portout "/nav/status" as nav_status
   }

   node "Computer Vision" as vision {
     portin "/camera/image_raw" as vision_image
     portout "/vision/detections" as vision_detections
     portout "/vision/features" as vision_features
   }

   node "SLAM" as slam {
     portin "/odom" as slam_odom
     portin "/vision/features" as slam_features
     portout "/map" as slam_map
     portout "/pose" as slam_pose
   }

   frontend_goal --> sm_goal
   sm_current --> frontend_status
   sm_goal --> nav_goal
   nav_odom --> slam_odom
   nav_status --> frontend_status
   vision_image --> vision_detections
   vision_features --> slam_features
   slam_map --> navigation
   slam_pose --> navigation
   @enduml

Deployment Architecture
========================

.. uml::

   @startuml Deployment
   !theme plain
   skinparam node {
     backgroundColor<<docker>> LightBlue
     backgroundColor<<system>> LightGreen
   }

   folder "Docker Host" as host {
     card "Frontend Container" <<docker>> as frontend
     card "Autonomy Container" <<docker>> as autonomy
     card "SLAM Container" <<docker>> as slam
     card "Database Container" <<docker>> as database
   }

   card "Nginx Proxy" <<system>> as nginx
   card "ROS2 Network" <<system>> as ros2
   card "Web Browser" <<system>> as browser

   browser --> nginx : HTTPS/WSS
   nginx --> frontend : HTTP/WS
   frontend --> autonomy : ROS2 Topics
   frontend --> slam : ROS2 Topics
   autonomy --> database : Data persistence
   slam --> database : Map storage
   autonomy --> ros2 : DDS communication
   slam --> ros2 : DDS communication

   note right of nginx
     Load balancer and
     SSL termination
   end note

   note right of ros2
     DDS discovery server
     for ROS2 node communication
   end note
   @enduml


