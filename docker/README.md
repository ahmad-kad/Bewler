# Universal Docker Setup for URC 2026

This directory contains the universal Docker configuration for the URC 2026 Robotics Platform. The setup provides consistent, reproducible environments for development and production deployment.

## 🚀 Quick Start

### Development Environment
```bash
# Start all services for development
docker-compose -f docker-compose.universal.yml up

# Or start specific services
docker-compose -f docker-compose.universal.yml up safety-service navigation-service
```

### Production Environment
```bash
# Start production deployment
docker-compose -f docker-compose.prod.yml up

# With mock systems enabled
COMPOSE_PROFILES=mock docker-compose -f docker-compose.prod.yml up
```

## 📦 Available Services

### Core Autonomy Services
- **`state-management-service`** - ROS2 State Machine Director
- **`safety-service`** - Safety monitoring and emergency systems
- **`navigation-service`** - GPS navigation and path planning
- **`vision-service`** - Computer vision with ArUco detection
- **`mission-service`** - Mission execution and coordination

### Supporting Services
- **`websocket-bridge-service`** - WebSocket to ROS2 bridge
- **`can-mock-service`** - CAN bus mock simulator (⚠️ MOCK DATA)
- **`sensor-bridge-service`** - Hardware sensor interface
- **`slam-service`** - SLAM mapping and localization
- **`led-service`** - Status LED control

### User Interface
- **`frontend-service`** - React development server
- **`frontend-prod`** - Nginx production server

### Development Tools
- **`development`** - Full development environment with all tools
- **`help`** - Shows available Docker targets

## 🏗️ Building Images

### Build All Services
```bash
# Build all services in parallel (recommended)
./scripts/build_universal_docker.sh --all

# Or build specific services
./scripts/build_universal_docker.sh safety-service navigation-service frontend-prod
```

### Build Options
```bash
# Debug build for development
./scripts/build_universal_docker.sh --debug --all

# Push to registry after building
./scripts/build_universal_docker.sh --push --registry myregistry.com/urc2026 frontend-prod

# Serial build (one at a time)
./scripts/build_universal_docker.sh --serial safety-service
```

### Manual Docker Build
```bash
# Build specific service
docker build --target safety-service -t urc2026:safety-service .

# Build with build arguments
docker build \
  --build-arg BUILDKIT_INLINE_CACHE=1 \
  --build-arg BUILD_DATE=$(date -u +'%Y-%m-%dT%H:%M:%SZ') \
  --target safety-service \
  -t urc2026:safety-service \
  -f docker/Dockerfile.universal .
```

## 🔧 Configuration

### Environment Variables

#### Production Environment (`.env` file)
```bash
# ROS2 Configuration
ROS_DOMAIN_ID=42

# Frontend Configuration
FRONTEND_PORT=8080

# Service Profiles
COMPOSE_PROFILES=mock,monitoring

# Registry Configuration (if pushing images)
REGISTRY_PREFIX=myregistry.com/urc2026
```

#### Development Environment
All services use default development settings:
- `URC_ENV=development`
- `ROS_DOMAIN_ID=42`
- Mock systems enabled by default

### Volume Mounts

#### Configuration
- `./config:/home/urc/config:ro` - Read-only configuration files

#### Logs (Production)
- `ros2_prod_logs:/home/urc/.ros/log` - ROS2 logging persistence

#### Development
- `./frontend/src:/app/src` - Frontend source code hot-reload
- `./frontend/public:/app/public` - Frontend assets

## 🌐 Networking

### Development Network
- **Network**: `urc-network`
- **Subnet**: `172.20.0.0/16`
- **Services**: All ports exposed for development

### Production Network
- **Network**: `urc-prod-network`
- **Subnet**: `172.21.0.0/16`
- **Security**: Most ports bound to localhost only
- **External Access**: Only frontend and monitoring (when enabled)

### Service Ports

| Service | Port | Development | Production |
|---------|------|-------------|------------|
| Frontend | 3000/80 | Exposed | Configurable |
| WebSocket Bridge | 8765 | Exposed | Localhost only |
| CAN Mock | 8766 | Exposed | Localhost only |
| Monitoring | 9090 | N/A | Localhost only |

## 🔍 Health Checks

All services include health checks for reliability:

### Timing
- **Interval**: 10-60 seconds (service-dependent)
- **Timeout**: 3-15 seconds
- **Retries**: 3-5 attempts
- **Start Period**: 10-30 seconds

### Health Check Commands
```bash
# ROS2 Services
ros2 node list | grep -q <service_name>

# Python Services
python3 -c "import <module>; print('OK')"

# Web Services
curl -f http://localhost:<port>
```

## 🛠️ Troubleshooting

### Common Issues

#### Build Failures
```bash
# Check build logs
docker build --progress=plain --target <service> .

# Clean build cache
docker builder prune -f
```

#### Service Won't Start
```bash
# Check service logs
docker-compose logs <service_name>

# Check health status
docker-compose ps

# Manual health check
docker exec <container_name> <health_check_command>
```

#### Network Issues
```bash
# Check network connectivity
docker network ls
docker network inspect urc-network

# Restart network
docker-compose down
docker-compose up --scale <service>=0 --scale <service>=1
```

#### Resource Issues
```bash
# Check resource usage
docker stats

# Adjust resource limits in docker-compose.prod.yml
deploy:
  resources:
    limits:
      memory: 512M
      cpus: '1.0'
```

### Debug Mode

Enable debug logging for troubleshooting:

```bash
# Set environment variables
export DOCKER_BUILDKIT=1
export BUILDKIT_PROGRESS=plain

# Run with debug output
docker-compose --verbose up <service>
```

## 📊 Monitoring

### Production Monitoring
Enable the monitoring profile for health dashboards:

```bash
COMPOSE_PROFILES=monitoring docker-compose -f docker-compose.prod.yml up
```

Access monitoring at `http://localhost:9090`

### Log Aggregation
```bash
# View all service logs
docker-compose logs -f

# View specific service logs
docker-compose logs -f safety-service

# Export logs for analysis
docker-compose logs > system_logs.txt
```

## 🔒 Security Considerations

### Production Security
- Services run as non-root user (`urc`)
- Most ports bound to localhost only
- Mock systems clearly labeled and not enabled by default
- Resource limits prevent resource exhaustion
- Health checks prevent silent failures

### Development Security
- All services accessible for debugging
- Mock data clearly identified
- No sensitive data in containers

## 📚 Advanced Usage

### Custom Build Arguments
```dockerfile
# In Dockerfile.universal
ARG BUILD_TYPE=release
ARG GIT_COMMIT=unknown
ARG BUILD_DATE=unknown

# Use in builds
docker build \
  --build-arg BUILD_TYPE=debug \
  --build-arg GIT_COMMIT=$(git rev-parse HEAD) \
  .
```

### Multi-Stage Build Optimization
```dockerfile
# Base stage with common dependencies
FROM ros:humble-ros-base AS base
RUN apt-get update && apt-get install -y common-deps

# Service-specific stages inherit from base
FROM base AS safety-service
RUN apt-get install -y safety-specific-deps
```

### Registry Management
```bash
# Login to registry
docker login myregistry.com

# Tag and push
docker tag urc2026:safety-service myregistry.com/urc2026:safety-service
docker push myregistry.com/urc2026:safety-service

# Pull for deployment
docker pull myregistry.com/urc2026:safety-service
```

## 🤝 Contributing

When adding new services:

1. Add service target to `Dockerfile.universal`
2. Update `docker-compose.universal.yml`
3. Update `docker-compose.prod.yml` (if needed)
4. Add to `SERVICES` array in `build_universal_docker.sh`
5. Update this README

### Service Template
```dockerfile
FROM base AS new-service
COPY --chown=urc path/to/service ./service/
HEALTHCHECK --interval=30s --timeout=10s --start-period=15s --retries=3 \
    CMD python3 -c "import service; print('OK')"
CMD ["python3", "-m", "service.main"]
```

## 📞 Support

For Docker-related issues:
1. Check service logs: `docker-compose logs <service>`
2. Verify configuration: `docker-compose config`
3. Test individual builds: `./scripts/build_universal_docker.sh <service>`
4. Check network connectivity: `docker network inspect urc-network`



