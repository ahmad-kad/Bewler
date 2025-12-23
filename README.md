# Bewler

URC 2026 Robotics Calibration System - Professional camera calibration toolkit for autonomous robotics.

## Overview

Bewler provides a comprehensive camera calibration system designed for robotics applications, specifically targeting the University Rover Challenge (URC) 2026 competition. The system supports multiple camera types, sensor auto-detection, and provides both programmatic APIs and command-line tools.

## Features

-  **Automatic Sensor Detection** - Supports IMX219, IMX477, IMX708, OV5647 cameras
-  **Multiple Calibration Methods** - ChArUco board and ArUco tag calibration
-  **Clean Architecture** - Modular design with proper separation of concerns
-  **Comprehensive Testing** - Full test coverage with professional tooling
-  **Easy Installation** - Python package with simple CLI tools
-  **Production Ready** - Zero linting errors, type-safe, well-documented

## Project Structure

```
bewler/
 calibration/           # Main calibration system
    src/              # Core library modules
 auto_calibrate.py     # Main calibration tool
    tests/            # Unit and integration tests
    docs/             # Documentation
    tools/            # Setup and configuration
 .gitignore           # Python/CV project exclusions
 README.md            # This file
```

## Quick Start

### Installation

```bash
git clone https://github.com/ahmad-kad/Bewler.git
cd Bewler/calibration
pip install -r tools/requirements.txt
```

### Basic Usage

```bash
# Calibrate a camera with auto sensor detection
python3 auto_calibrate.py --camera 0

# List all calibrated cameras
python3 auto_calibrate.py --list-cameras

# Generate ArUco tags for your project
python -c "from src.aruco.generator import ArucoGenerator; gen = ArucoGenerator(); gen.save_sheet_png([1.0, 2.0, 5.0], 'tags.png')"
```

## Development

### Quality Checks

```bash
# Run all linting and type checking
cd calibration
flake8 .          # Style checking
black --check .   # Format checking
mypy .            # Type checking
vulture .         # Dead code detection
```

### Testing

```bash
# Run the test suite
cd calibration
python -m pytest tests/
```

## Documentation

See `calibration/docs/README.md` for comprehensive documentation including:
- Detailed API reference
- Calibration procedures
- Configuration options
- Troubleshooting guide

## License

This project is part of the URC 2026 Robotics Team efforts.

## Contributing

1. Follow the established code quality standards
2. Add tests for new functionality
3. Update documentation as needed
4. Run all quality checks before committing

