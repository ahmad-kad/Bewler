#!/usr/bin/env python3
"""
Sensor Bridge Node Launcher

Launches the WebSocket-to-ROS2 sensor bridge node.
"""

import sys
import os

# Add the package to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from sensor_bridge.websocket_bridge import main

if __name__ == '__main__':
    main()
