# Installation with pyproject.toml

This project now uses `pyproject.toml` for modern Python dependency management.

## Quick Start

### Basic Installation
```bash
# Install the package in editable mode
pip install -e .

# Or install with development dependencies
pip install -e .[dev]

# Or install with all optional dependencies
pip install -e .[full]
```

### Installation from Source
```bash
# Clone the repository
git clone <repository-url>
cd Bewler

# Install dependencies
pip install -e .[dev]
```

## Dependency Groups

### Core Dependencies (always installed)
- `numpy>=2.0.0` - Numerical computing
- `opencv-contrib-python-headless>=4.8.0` - Computer vision (headless)
- `packaging>=21.0` - Version handling
- `Pillow>=9.0.0` - Image processing

### Development Dependencies (`[dev]`)
- `pytest>=7.0.0` - Testing framework
- `pytest-asyncio>=0.21.0` - Async testing support
- `black>=22.0.0` - Code formatter
- `flake8>=4.0.0` - Linter
- `mypy>=0.950` - Type checker

### ROS 2 Dependencies (`[ros2]`)
Note: ROS 2 dependencies should be installed via system package manager or ROS 2 workspace, not via pip.

## Command-Line Tools

After installation, these commands are available:

- `aruco-servoing` - Run ArUco detection for camera servoing
- `aruco-generate` - Generate ArUco marker tags
- `aruco-validate` - Validate ArUco markers
- `charuco-generate` - Generate ChArUco calibration boards
- `charuco-calibrate` - Calibrate camera using ChArUco

## Migration from requirements.txt

The old `tools/requirements.txt` is still available for reference, but new installations should use:

```bash
pip install -e .[dev]
```

instead of:

```bash
pip install -r tools/requirements.txt
```

## Benefits of pyproject.toml

1. **Standard Python packaging** - Follows PEP 517/518 standards
2. **Dependency groups** - Separate core, dev, and optional dependencies
3. **Entry points** - CLI tools automatically available after installation
4. **Tool configuration** - Black, mypy, pytest configs in one place
5. **Modern workflow** - Compatible with pip, poetry, and other modern tools
