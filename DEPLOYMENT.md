# 🚀 Production Deployment Guide

This guide covers deploying the URC 2026 Robotics Platform to production environments.

## Prerequisites

### System Requirements
- **OS**: Ubuntu 22.04 LTS or equivalent
- **CPU**: 4+ cores (8+ recommended)
- **RAM**: 8GB minimum (16GB+ recommended)
- **Storage**: 50GB+ available space
- **Network**: Gigabit Ethernet, ROS2-compatible network setup

### Software Dependencies
- Docker 24.0+
- Docker Compose 2.0+
- Python 3.10+
- ROS2 Humble Hawksbill
- Git LFS (for large model files)

## Quick Deployment

### 1. Clone and Setup
```bash
# Clone repository
git clone https://github.com/your-org/urc-machiato-2026.git
cd urc-machiato-2026

# Install Python dependencies
pip install -e .

# Install development tools (optional)
pip install -e ".[dev]"

# Install documentation tools (optional)
pip install -e ".[docs]"
```

### 2. Environment Configuration
```bash
# Copy production configuration
cp config/production.yaml config/local.yaml

# Edit configuration for your environment
nano config/local.yaml
```

Required environment variables:
```bash
export ROS_DOMAIN_ID=42                    # Unique ROS2 domain
export ROS_DISCOVERY_SERVER=192.168.1.100:11811  # Discovery server
export WEBSOCKET_URL=ws://your-sensor-bridge:8080
export CAMERA_FRONT_URL=http://camera-front:8080
export CAMERA_REAR_URL=http://camera-rear:8080
```

### 3. Validate Configuration
```bash
# Validate configuration
python scripts/validate_config.py

# Should show: "Configuration is production-ready!"
```

### 4. Deploy Services

#### Option A: Docker Compose (Recommended)
```bash
# Build all services
docker-compose -f docker-compose.prod.yml build

# Start services
docker-compose -f docker-compose.prod.yml up -d

# Check status
docker-compose -f docker-compose.prod.yml ps
```

#### Option B: Manual ROS2 Deployment
```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Build autonomy packages
cd Autonomy/ros2_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Start core services
source install/setup.bash
ros2 launch autonomy_system system.launch.py
```

## Service Architecture

### Core Services
- **Autonomy System**: Main robotics control (`autonomy_system`)
- **Sensor Bridge**: Hardware interface (`sensor_bridge`)
- **SLAM**: Localization and mapping (`slam_system`)
- **Computer Vision**: Object detection (`computer_vision`)
- **Mission Control**: Task execution (`mission_control`)
- **Web Interface**: User control panel (`frontend`)

### Network Services
- **ROS2 Discovery Server**: Service discovery
- **WebSocket Bridge**: Real-time data streaming
- **API Gateway**: RESTful interfaces

## Monitoring and Observability

### Health Checks
```bash
# Check service health
curl http://localhost:8080/health

# ROS2 diagnostics
ros2 topic echo /diagnostics

# System logs
docker-compose logs -f autonomy_system
```

### Performance Monitoring
- ROS2 `/performance` topic
- Prometheus metrics (if configured)
- System resource monitoring

## Troubleshooting

### Common Issues

#### ROS2 Network Issues
```bash
# Check ROS2 domain
echo $ROS_DOMAIN_ID

# Verify discovery server
ros2 multicast receive
```

#### Hardware Connection Issues
```bash
# Test sensor connections
ros2 topic list | grep sensor

# Check WebSocket bridge
curl http://localhost:8080/status
```

#### Performance Issues
```bash
# Monitor CPU/memory usage
docker stats

# Check ROS2 performance
ros2 topic hz /odom
```

### Logs and Debugging
```bash
# View service logs
docker-compose logs autonomy_system

# Enable debug logging
export ROS_LOG_LEVEL=DEBUG
export AUTONOMY_LOG_LEVEL=DEBUG

# Restart with debug
docker-compose restart autonomy_system
```

## Backup and Recovery

### Configuration Backup
```bash
# Backup configurations
tar -czf backup_configs_$(date +%Y%m%d).tar.gz config/

# Backup data
tar -czf backup_data_$(date +%Y%m%d).tar.gz data/
```

### Service Recovery
```bash
# Restart all services
docker-compose restart

# Rollback to previous version
docker-compose pull && docker-compose up -d
```

## Security Considerations

### Network Security
- ROS2 uses DDS security when configured
- WebSocket connections should use WSS in production
- API endpoints require authentication

### Access Control
- Limit physical access to deployment hardware
- Use VPN for remote administration
- Implement proper user authentication

### Data Protection
- Encrypt sensitive configuration data
- Secure ROS2 keystore files
- Regular security updates

## Performance Tuning

### ROS2 Optimization
```yaml
# config/production.yaml
network:
  qos_depth_sensor: 1      # Minimize latency
  qos_depth_command: 5     # Buffer commands
  qos_depth_status: 1      # Latest status only
```

### Hardware Optimization
- Use real-time kernel for robotics
- Configure CPU affinity for critical processes
- Optimize network buffer sizes

## Maintenance

### Regular Tasks
- **Daily**: Monitor system health and logs
- **Weekly**: Update dependencies and security patches
- **Monthly**: Full system backup and performance review

### Updates
```bash
# Update codebase
git pull origin main

# Update dependencies
pip install -e . --upgrade

# Rebuild containers
docker-compose build --no-cache
docker-compose up -d
```

## Support

For deployment issues:
1. Check this documentation
2. Review system logs
3. Run configuration validation: `python scripts/validate_config.py`
4. Check GitHub Issues for known problems
5. Contact the development team

---

**Last updated**: $(date)
**Version**: 1.0.0




