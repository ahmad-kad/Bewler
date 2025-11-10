# URC 2026 Documentation

This directory contains the complete Sphinx-based documentation system for the URC 2026 Mars Rover Autonomy System.

## 📚 Documentation Overview

The documentation is organized into multiple sections covering all aspects of the project:

- **Python API**: Core autonomy algorithms, calibration tools, and utilities
- **C++ API**: ROS2 interfaces and message definitions
- **JavaScript/TypeScript API**: React frontend components and hooks
- **System Architecture**: High-level design and component interactions
- **Calibration**: Camera, hand-eye, and sensor calibration procedures
- **Development**: Contributing guidelines, testing, and deployment

## 🚀 Quick Start

### Prerequisites

- Python 3.10+
- Node.js 18+
- Doxygen (for C++ documentation)
- JSDoc (for JavaScript documentation)

### Installation

```bash
# Install Python documentation dependencies
cd docs
pip install -r requirements.txt

# Install JavaScript documentation tools
npm install -g jsdoc

# Install system dependencies (Ubuntu/Debian)
sudo apt-get install doxygen graphviz
```

### Building Documentation

```bash
# Build all documentation types
make all

# Build only specific types
make html          # Sphinx HTML documentation
make doxygen       # C++ Doxygen documentation
make jsdoc         # JavaScript JSDoc documentation

# Clean all builds
make clean-all

# Serve locally for development
make serve         # Starts HTTP server on port 8000
```

## 📖 Documentation Structure

```
docs/
├── conf.py                 # Sphinx configuration
├── index.rst              # Main documentation index
├── requirements.txt       # Python dependencies
├── Makefile              # Build automation
├── Doxyfile             # Doxygen configuration for C++
├── jsdoc.json           # JSDoc configuration for JS/TS
├── _static/             # Static assets (CSS, images)
├── _templates/          # Sphinx templates
├── _build/              # Generated documentation (not committed)
├── api/                 # API reference documentation
│   ├── python/         # Python API docs
│   ├── cpp/            # C++ API docs
│   └── javascript/     # JavaScript/TypeScript API docs
├── architecture/        # System architecture docs
├── subsystems/          # Subsystem-specific docs
├── calibration/         # Calibration procedures
├── development/         # Development guidelines
└── reference/           # Reference materials
```

## ✍️ Writing Documentation

### Python Documentation

Use Google-style docstrings:

```python
def calibrate_camera(image_points, object_points):
    """Calibrate camera using corresponding 3D object and 2D image points.

    Args:
        image_points (np.ndarray): 2D points in image coordinates
        object_points (np.ndarray): Corresponding 3D points in world coordinates

    Returns:
        tuple: (camera_matrix, dist_coeffs, rvecs, tvecs)

    Raises:
        ValueError: If input arrays have incompatible shapes
    """
```

### C++ Documentation

Use Doxygen comments:

```cpp
/**
 * @brief Calibrate camera intrinsics from chessboard images
 *
 * This function estimates camera intrinsic parameters using multiple
 * views of a chessboard pattern.
 *
 * @param images Vector of calibration images
 * @param board_size Size of the chessboard (corners per row/column)
 * @param square_size Physical size of chessboard squares
 * @return CameraCalibrationResult containing intrinsic parameters
 * @throws CalibrationException if calibration fails
 */
CameraCalibrationResult calibrateCameraIntrinsics(
    const std::vector<cv::Mat>& images,
    const cv::Size& board_size,
    float square_size);
```

### JavaScript/TypeScript Documentation

Use JSDoc comments:

```javascript
/**
 * ROS Connection Hook for rover communication
 *
 * @param {Object} config - Connection configuration
 * @param {string} [config.url='ws://localhost:9090'] - ROS bridge URL
 * @param {number} [config.reconnectInterval=3000] - Reconnection interval
 * @returns {Object} Connection state and control functions
 *
 * @example
 * const { isConnected, ros } = useROS({
 *   url: 'ws://192.168.1.100:9090'
 * });
 */
export function useROS(config = {}) {
  // Implementation...
}
```

## 🔧 Development Workflow

### Adding New Documentation

1. **Create RST files** in appropriate directories
2. **Add to toctree** in parent index files
3. **Build and test** locally: `make html && make serve`
4. **Commit changes** with descriptive messages

### Documentation Standards

- Use **semantic line breaks** for better diffs
- **Cross-reference** related sections with `:ref:` and `:doc:`
- **Include examples** for complex APIs
- **Keep screenshots updated** when UI changes
- **Version documentation** with code changes

### Code Documentation

- **Document public APIs** comprehensively
- **Include type hints** in Python (mypy compatible)
- **Document exceptions** that can be raised
- **Provide usage examples** in docstrings
- **Keep documentation current** with code changes

## 🚀 Deployment

### Local Development

```bash
# Auto-rebuild on changes (requires sphinx-autobuild)
make dev

# Check for broken links
make linkcheck

# Validate doctests
make doctest
```

### CI/CD

Documentation is automatically built and deployed via GitHub Actions:

- **Push to main**: Deploys to GitHub Pages
- **Pull requests**: Validates documentation builds
- **Path filtering**: Only builds when docs or code changes

### Manual Deployment

```bash
# Build for production
make github-pages

# Deploy to custom domain
# Update CNAME file and configure DNS
```

## 📊 Quality Checks

### Automated Checks

- **Link validation**: All internal/external links working
- **Syntax validation**: RST and Python syntax correct
- **Doctest execution**: Code examples run successfully
- **Coverage reporting**: API documentation completeness

### Manual Reviews

- **Content accuracy**: Documentation matches implementation
- **Clarity**: Instructions are clear and complete
- **Examples**: Code examples work as documented
- **Navigation**: Logical structure and cross-references

## 🆘 Troubleshooting

### Common Issues

**Import errors in Sphinx build:**
```bash
# Ensure all Python dependencies are installed
pip install -r requirements.txt

# Check Python path includes project root
export PYTHONPATH="${PYTHONPATH}:.."
```

**Doxygen build fails:**
```bash
# Verify Doxygen installation
doxygen --version

# Check Doxyfile configuration
make doxygen 2>&1 | head -20
```

**JSDoc build fails:**
```bash
# Verify Node.js and JSDoc
node --version
jsdoc --version

# Check JSDoc configuration
make jsdoc 2>&1
```

### Getting Help

- **Documentation issues**: Check existing issues or create new ones
- **Build problems**: Review CI logs for error details
- **Content questions**: Consult the main project documentation

## 📈 Contributing

See the main project CONTRIBUTING.md for general contribution guidelines.

### Documentation Contributions

1. **Follow style guides** for each language
2. **Test builds locally** before submitting
3. **Include examples** for new features
4. **Update related docs** when changing APIs
5. **Review changes** for clarity and completeness

## 📞 Support

For questions about the documentation system, contact the development team or create an issue with the "documentation" label.

## 📄 License

This documentation is part of the URC 2026 Mars Rover Autonomy System project.
