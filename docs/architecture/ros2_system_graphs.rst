===================
ROS2 System Graphs
===================

This document contains automatically generated graphs of the ROS2 system components, nodes, topics, and services.

.. note::
   These diagrams are automatically generated from the running ROS2 system when the documentation is built. If ROS2 is not running, placeholder diagrams will be shown.

System Nodes Graph
==================

The following diagram shows all ROS2 nodes currently running in the system:

.. ros2-nodes::
   :output: ros2_system_nodes.png
   :title: ROS2 System Nodes

Topic Network Graph
===================

This diagram visualizes the ROS2 topic network and message flow:

.. ros2-topics::
   :output: ros2_system_topics.png
   :title: ROS2 Topic Network

Services Graph
==============

Available ROS2 services in the system:

.. ros2-services::
   :output: ros2_system_services.png
   :title: ROS2 Services

Understanding ROS2 Graphs
=========================

Nodes
-----

ROS2 nodes are the fundamental computational units in a ROS2 system. Each node:

- Performs a specific computation task
- Communicates via topics, services, and actions
- Can be written in different programming languages
- Runs in its own process

The nodes graph shows the hierarchical organization and relationships between different system components.

Topics
------

Topics enable asynchronous communication between nodes:

- **Publishers** send messages on topics
- **Subscribers** receive messages from topics
- Topics are typed (each topic has a specific message type)
- Multiple publishers/subscribers can use the same topic

The topics graph helps visualize the data flow architecture of the system.

Services
--------

Services provide synchronous request-response communication:

- **Clients** send requests to services
- **Servers** process requests and return responses
- Services are used for configuration, status queries, and control commands
- Each service call blocks until a response is received

The services graph shows the available control and configuration interfaces.

Automated Generation
====================

These graphs are generated automatically using the following process:

1. **Introspection**: Query the running ROS2 system using ``ros2`` command-line tools
2. **Data Collection**: Gather information about nodes, topics, and services
3. **Diagram Generation**: Create PlantUML diagrams from the collected data
4. **Rendering**: Convert PlantUML to images during Sphinx build

This ensures that the documentation always reflects the current system state and architecture.


