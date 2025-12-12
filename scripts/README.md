# Scripts Directory

Utility scripts for development, testing, and production deployment.

## Production Scripts

### Core Validation
- **`validate_config.py`** - Validates configuration files for production readiness
- **`production_health_check.py`** - Comprehensive system health check for deployment

### GitHub Integration
- **`extract-todos-to-issues.py`** - Automated TODO extraction and GitHub issue creation

### Documentation
- **`convert_md_to_rst.py`** - Converts Markdown to reStructuredText for Sphinx
- **`build_docs.sh`** - Builds documentation (requires docs dependencies)

## Development Scripts (Autonomy/scripts/)

### System Management
- **`system_launch.sh`** - Launch complete autonomy system
- **`system_stop.sh`** - Stop all running services
- **`health_check.sh`** - Basic system health monitoring

### Testing & Validation
- **`test_full_system.sh`** - End-to-end system testing
- **`test_simulation.sh`** - Simulation-only testing
- **`validate_deploy.sh`** - Deployment validation
- **`validate_infrastructure.py`** - Infrastructure validation

### Development Tools
- **`check_code.sh`** - Code quality checks
- **`dev_test.sh`** - Development testing
- **`benchmark_system.sh`** - Performance benchmarking
- **`daily_velocity_check.py`** - Development velocity tracking

### Competition Scripts
- **`start_safety_testing.sh`** - Safety system testing for competition
- **`stop_safety_testing.sh`** - Stop safety testing
- **`navigate.sh`** - Navigation testing

## Usage

### Production Deployment
```bash
# Validate configuration
python scripts/validate_config.py

# Health check
python scripts/production_health_check.py

# Extract TODOs to GitHub issues
python scripts/extract-todos-to-issues.py
```

### Development
```bash
# Launch system
./Autonomy/scripts/system_launch.sh

# Run tests
./Autonomy/scripts/test_full_system.sh

# Health check
./Autonomy/scripts/health_check.sh
```

## Notes

- Production scripts are validated and tested for deployment
- Development scripts may require ROS2 environment and hardware
- Some scripts require specific dependencies (see pyproject.toml)




