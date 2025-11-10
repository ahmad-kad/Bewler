# State Machine Quick Start

Get the rover state machine running in 3 minutes!

## 🚀 Quick Launch

```bash
# 1. Go to your ROS2 workspace
cd ~/ros2_ws  # Adjust path as needed

# 2. Build (if not already built)
colcon build --packages-select autonomy_interfaces autonomy_state_machine

# 3. Source and launch
source install/setup.bash
ros2 launch autonomy_state_machine state_machine.launch.py
```

**That's it!** The state machine will boot up and transition to IDLE state automatically.

## 🎮 Basic Control

### Check Current State
```bash
ros2 service call /state_machine/get_system_state autonomy_interfaces/srv/GetSystemState
```

### Request Autonomous Mission
```bash
ros2 service call /state_machine/change_state autonomy_interfaces/srv/ChangeState "{
  desired_state: 'AUTONOMOUS',
  desired_substate: 'SCIENCE',
  reason: 'Starting science mission',
  operator_id: 'operator'
}"
```

### Return to Safe State
```bash
ros2 service call /state_machine/change_state autonomy_interfaces/srv/ChangeState "{
  desired_state: 'IDLE',
  reason: 'Mission complete',
  operator_id: 'operator'
}"
```

## 📊 Monitor State Changes

```bash
# Watch state updates (10Hz)
ros2 topic echo /state_machine/current_state
```

## 🧪 Test Everything Works

```bash
# Run tests
colcon test --packages-select autonomy_state_machine
```

## 📖 Need More Help?

- **Full Documentation**: See [README.md](README.md)
- **API Reference**: Check the API Reference section in README
- **Troubleshooting**: See Troubleshooting section in README

---

**Next**: Integrate with your rover's frontend or mission control system!
