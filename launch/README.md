# Launch Directory

ROS2 launch files for starting different system components.

## Launch Files

- `mission_system.launch.py` - Launch the complete mission execution system
- `rover_simulation.launch.py` - Launch rover simulation with Gazebo
- `slam_bridge.launch.py` - Launch SLAM and mapping components

## Usage

```bash
# Launch complete system
ros2 launch launch/mission_system.launch.py

# Launch simulation
ros2 launch launch/rover_simulation.launch.py

# Launch SLAM components
ros2 launch launch/slam_bridge.launch.py
```




