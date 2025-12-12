#!/usr/bin/env python3
"""
Sensor Bridge Node Launcher

Launches the WebSocket-to-ROS2 sensor bridge node.
"""

import os
import sys

from sensor_bridge.websocket_bridge import main

# Add the package to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))


if __name__ == "__main__":
    main()
