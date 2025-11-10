#!/bin/bash

# Deployment Readiness Check Script
# Validates system readiness for hardware deployment

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
ROS_WS="$PROJECT_ROOT/ros2_ws"
DEPLOYMENT_TYPE=${1:-competition}

# Check results
ISSUES=0
WARNINGS=0
PASSED=0

# Function to print colored output
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[✓]${NC} $1"
    ((PASSED++))
}

print_warning() {
    echo -e "${YELLOW}[⚠]${NC} $1"
    ((WARNINGS++))
}

print_error() {
    echo -e "${RED}[✗]${NC} $1"
    ((ISSUES++))
}

print_header() {
    echo ""
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}$(printf '%.0s=' {1..60})${NC}"
}

# Function to check system environment
check_system_environment() {
    print_header "System Environment Check"

    # Check OS and version
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        local os_version=$(lsb_release -d 2>/dev/null | cut -f2 || echo "Unknown Linux")
        print_success "Operating System: $os_version"
    else
        print_error "Unsupported OS: $OSTYPE"
    fi

    # Check ROS2 version
    if command -v ros2 &> /dev/null; then
        local ros_version=$(ros2 --version 2>/dev/null | head -1 || echo "Unknown")
        print_success "ROS2 Version: $ros_version"

        # Check if ROS_DOMAIN_ID is set for multi-robot scenarios
        if [ -n "$ROS_DOMAIN_ID" ]; then
            print_success "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
        else
            print_info "ROS_DOMAIN_ID not set (OK for single robot)"
        fi
    else
        print_error "ROS2 not installed or not in PATH"
    fi

    # Check Python version
    local python_version=$(python3 --version 2>&1 | cut -d' ' -f2 || echo "Unknown")
    print_success "Python Version: $python_version"

    # Check available memory
    local total_mem=$(free -h | grep "^Mem:" | awk '{print $2}')
    local avail_mem=$(free -h | grep "^Mem:" | awk '{print $7}')
    print_success "Memory: ${avail_mem} available of ${total_mem} total"

    # Check disk space
    local disk_free=$(df -h "$PROJECT_ROOT" | tail -1 | awk '{print $4}')
    local disk_usage=$(df "$PROJECT_ROOT" | tail -1 | awk '{print $5}' | sed 's/%//')

    if [ "$disk_usage" -lt 90 ]; then
        print_success "Disk Space: ${disk_free} available"
    else
        print_error "Low disk space: ${disk_free} available"
    fi
}

# Function to check ROS2 workspace
check_ros_workspace() {
    print_header "ROS2 Workspace Validation"

    # Check workspace structure
    if [ ! -d "$ROS_WS/src" ]; then
        print_error "ROS2 workspace src directory missing"
        return 1
    fi
    print_success "ROS2 workspace structure valid"

    # Check if workspace is built
    if [ ! -f "$ROS_WS/install/setup.bash" ]; then
        print_error "ROS2 workspace not built"
        echo "  Run: cd ros2_ws && colcon build"
        return 1
    fi
    print_success "ROS2 workspace built"

    # Check package installation
    local installed_packages=$(ros2 pkg list 2>/dev/null | wc -l)
    if [ "$installed_packages" -gt "0" ]; then
        print_success "Packages installed: $installed_packages"
    else
        print_error "No ROS2 packages installed"
    fi

    # Check autonomy packages specifically
    local autonomy_packages=$(ros2 pkg list 2>/dev/null | grep -c autonomy || echo "0")
    if [ "$autonomy_packages" -gt "0" ]; then
        print_success "Autonomy packages: $autonomy_packages"
    else
        print_error "No autonomy packages found"
    fi
}

# Function to check hardware interfaces
check_hardware_interfaces() {
    print_header "Hardware Interface Validation"

    # Check network interfaces
    local network_interfaces=$(ip -br addr show 2>/dev/null | wc -l)
    if [ "$network_interfaces" -gt "0" ]; then
        print_success "Network interfaces available: $network_interfaces"

        # Check for common interface names
        if ip addr show | grep -q "eth0\|wlan0\|enp"; then
            print_success "Ethernet/WiFi interfaces detected"
        else
            print_warning "No standard network interfaces found"
        fi
    else
        print_error "No network interfaces available"
    fi

    # Check USB devices (common for sensors)
    local usb_devices=$(lsusb 2>/dev/null | wc -l || echo "0")
    if [ "$usb_devices" -gt "0" ]; then
        print_success "USB devices detected: $usb_devices"
    else
        print_info "No USB devices detected (may be normal)"
    fi

    # Check serial ports (for legacy sensors)
    if [ -d "/dev/serial" ] || ls /dev/tty* 2>/dev/null | grep -q tty; then
        local serial_ports=$(ls /dev/tty* 2>/dev/null | wc -l || echo "0")
        print_success "Serial ports available: $serial_ports"
    else
        print_info "No serial ports detected (may be normal for modern sensors)"
    fi

    # Check GPU availability (for computer vision)
    if command -v nvidia-smi &> /dev/null; then
        local gpu_count=$(nvidia-smi --list-gpus 2>/dev/null | wc -l || echo "1")
        print_success "GPU available: $gpu_count GPU(s) detected"
    elif command -v glxinfo &> /dev/null && glxinfo 2>/dev/null | grep -q "OpenGL"; then
        print_success "Integrated graphics available"
    else
        print_warning "No GPU/graphics acceleration detected"
    fi
}

# Function to check autonomy system components
check_autonomy_components() {
    print_header "Autonomy System Components"

    # Source workspace
    cd "$ROS_WS"
    source install/setup.bash 2>/dev/null || true
    cd "$PROJECT_ROOT"

    # Check core subsystems
    local subsystems=("autonomy_state_management" "autonomy_navigation" "autonomy_computer_vision" "autonomy_safety_system")

    for subsystem in "${subsystems[@]}"; do
        if ros2 pkg list 2>/dev/null | grep -q "$subsystem"; then
            print_success "Subsystem package: $subsystem"
        else
            print_error "Missing subsystem: $subsystem"
        fi
    done

    # Check launch files exist
    local launch_files=(
        "autonomy_state_management/state_machine.launch.py"
        "autonomy_navigation/navigation.launch.py"
        "autonomy_computer_vision/computer_vision.launch.py"
        "autonomy_safety_system/safety_system.launch.py"
    )

    for launch_file in "${launch_files[@]}"; do
        if [ -f "$ROS_WS/install/share/$launch_file" ]; then
            print_success "Launch file: $launch_file"
        else
            print_error "Missing launch file: $launch_file"
        fi
    done
}

# Function to perform quick system test
perform_quick_test() {
    print_header "Quick System Functionality Test"

    print_info "Starting quick autonomy system test..."

    # Try to launch state management (minimal test)
    timeout 10 ros2 launch autonomy_state_management state_machine.launch.py &
    local launch_pid=$!

    sleep 3

    # Check if any nodes started
    local node_count=$(ros2 node list 2>/dev/null | wc -l || echo "0")
    if [ "$node_count" -gt "0" ]; then
        print_success "System can start nodes ($node_count nodes)"

        # Check if topics are being published
        local topic_count=$(ros2 topic list 2>/dev/null | wc -l || echo "0")
        if [ "$topic_count" -gt "0" ]; then
            print_success "Topics being published ($topic_count topics)"
        else
            print_warning "No topics detected"
        fi
    else
        print_error "System cannot start nodes"
    fi

    # Cleanup
    kill $launch_pid 2>/dev/null || true
    pkill -f "state_machine" || true
    sleep 2
}

# Function to check configuration files
check_configuration() {
    print_header "Configuration Validation"

    # Check for config directories
    local config_dirs=(
        "autonomy_navigation/config"
        "autonomy_computer_vision/config"
        "autonomy_safety_system/config"
    )

    for config_dir in "${config_dirs[@]}"; do
        if [ -d "$ROS_WS/install/share/$config_dir" ]; then
            print_success "Config directory: $config_dir"

            # Check for YAML files
            local yaml_count=$(find "$ROS_WS/install/share/$config_dir" -name "*.yaml" 2>/dev/null | wc -l || echo "0")
            if [ "$yaml_count" -gt "0" ]; then
                print_success "  Config files: $yaml_count YAML file(s)"
            else
                print_warning "  No YAML config files found"
            fi
        else
            print_error "Missing config directory: $config_dir"
        fi
    done
}

# Function to check URC 2026 compliance
check_competition_compliance() {
    print_header "URC 2026 Competition Compliance"

    # Emergency stop time requirements
    print_info "Checking emergency stop requirements..."

    # Check if emergency stop service exists
    if ros2 service list 2>/dev/null | grep -q "software_estop\|emergency_stop"; then
        print_success "Emergency stop service available"
    else
        print_error "Emergency stop service missing"
    fi

    # Check safety system components
    if ros2 pkg list 2>/dev/null | grep -q "autonomy_safety_system"; then
        print_success "Safety system package present"
    else
        print_error "Safety system package missing"
    fi

    # Check for competition-specific configurations
    local competition_configs=$(find "$ROS_WS/install/share" -name "*competition*" -o -name "*urc*" 2>/dev/null | wc -l || echo "0")
    if [ "$competition_configs" -gt "0" ]; then
        print_success "Competition configurations found: $competition_configs"
    else
        print_info "No competition-specific configurations detected"
    fi

    # Check system resource requirements
    local total_mem_gb=$(free -g | grep "^Mem:" | awk '{print $2}')
    if [ "$total_mem_gb" -ge 8 ]; then
        print_success "Memory requirement met: ${total_mem_gb}GB"
    else
        print_warning "Memory below recommended: ${total_mem_gb}GB (8GB recommended)"
    fi
}

# Function to generate deployment report
generate_deployment_report() {
    print_header "Deployment Readiness Report"

    local report_file="deployment_check_report_$(date +%Y%m%d_%H%M%S).txt"

    cat > "$report_file" << EOF
URC 2026 Autonomy System Deployment Check Report
==============================================

Deployment Type: $DEPLOYMENT_TYPE
Timestamp: $(date)
System: $(hostname)

SUMMARY
=======
Checks Passed: $PASSED
Warnings: $WARNINGS
Critical Issues: $ISSUES

$(if [ $ISSUES -eq 0 ] && [ $WARNINGS -eq 0 ]; then
    echo "🎉 SYSTEM READY FOR DEPLOYMENT"
elif [ $ISSUES -eq 0 ]; then
    echo "⚠️  SYSTEM READY WITH WARNINGS"
else
    echo "❌ SYSTEM NOT READY FOR DEPLOYMENT"
fi)

SYSTEM ENVIRONMENT
==================
$(check_system_environment 2>/dev/null | grep -E "\[✓\]|\[⚠\]|\[✗\]" | sed 's/\x1b\[[0-9;]*m//g' || echo "Environment check failed")

ROS2 WORKSPACE
==============
$(check_ros_workspace 2>/dev/null | grep -E "\[✓\]|\[⚠\]|\[✗\]" | sed 's/\x1b\[[0-9;]*m//g' || echo "Workspace check failed")

HARDWARE INTERFACES
===================
$(check_hardware_interfaces 2>/dev/null | grep -E "\[✓\]|\[⚠\]|\[✗\]" | sed 's/\x1b\[[0-9;]*m//g' || echo "Hardware check failed")

AUTONOMY COMPONENTS
==================
$(check_autonomy_components 2>/dev/null | grep -E "\[✓\]|\[⚠\]|\[✗\]" | sed 's/\x1b\[[0-9;]*m//g' || echo "Component check failed")

COMPETITION COMPLIANCE
======================
$(check_competition_compliance 2>/dev/null | grep -E "\[✓\]|\[⚠\]|\[✗\]" | sed 's/\x1b\[[0-9;]*m//g' || echo "Compliance check failed")

DEPLOYMENT CHECKLIST
====================

Pre-Deployment Tasks:
$(if [ $ISSUES -gt 0 ]; then
    echo "❌ Fix all critical issues before deployment"
else
    echo "✅ All critical issues resolved"
fi)

$(if [ $WARNINGS -gt 0 ]; then
    echo "⚠️  Review warnings before deployment"
else
    echo "✅ No warnings to address"
fi)

Deployment Commands:
  # 1. Final system build
  cd Autonomy/ros2_ws && colcon build --continue-on-error

  # 2. System startup test
  ./Autonomy/scripts/health_check.sh

  # 3. Full system test
  ./Autonomy/scripts/test_full_system.sh --quick

  # 4. Competition launch (if applicable)
  ros2 launch rover_system competition.launch.py

Troubleshooting:
  • Run './scripts/health_check.sh' for quick diagnosis
  • Check ROS2 logs: 'ros2 run rqt_console rqt_console'
  • Validate network: 'ros2 run rqt_graph rqt_graph'
  • Test individual subsystems: './scripts/dev_test.sh'

Report generated by: $0
EOF

    print_success "Deployment report saved: $report_file"
}

# Function to show usage
usage() {
    echo "Usage: $0 [deployment_type]"
    echo ""
    echo "Deployment Readiness Check for URC 2026 Autonomy"
    echo ""
    echo "Deployment Types:"
    echo "  competition    URC 2026 competition deployment (default)"
    echo "  development    Development environment check"
    echo "  simulation     Simulation environment check"
    echo "  hardware       Hardware-only validation"
    echo ""
    echo "This script validates:"
    echo "  • System environment and requirements"
    echo "  • ROS2 workspace and packages"
    echo "  • Hardware interfaces and sensors"
    echo "  • Autonomy system components"
    echo "  • URC 2026 competition compliance"
    echo ""
    echo "Output:"
    echo "  • Colored console output with pass/warn/fail status"
    echo "  • Detailed deployment readiness report"
    echo "  • Actionable recommendations for issues"
}

# Main execution
main() {
    echo "🚀 URC 2026 Deployment Readiness Check"
    echo "======================================="
    echo "Deployment Type: ${DEPLOYMENT_TYPE:-competition}"
    echo ""

    # Run all checks
    check_system_environment
    check_ros_workspace
    check_hardware_interfaces
    check_autonomy_components
    check_configuration
    perform_quick_test
    check_competition_compliance

    # Generate report
    generate_deployment_report

    # Final summary
    print_header "Final Deployment Status"

    if [ $ISSUES -eq 0 ]; then
        if [ $WARNINGS -eq 0 ]; then
            print_success "🎉 SYSTEM IS READY FOR DEPLOYMENT!"
            print_info "All checks passed. System meets deployment requirements."
        else
            print_warning "⚠️  SYSTEM IS READY WITH WARNINGS"
            print_info "$WARNINGS warnings detected. Review before deployment."
        fi
    else
        print_error "❌ SYSTEM IS NOT READY FOR DEPLOYMENT"
        print_info "$ISSUES critical issues must be resolved before deployment."
        echo ""
        print_info "Run './scripts/health_check.sh' for quick diagnosis"
        exit 1
    fi

    echo ""
    echo "📊 Results: $PASSED passed, $WARNINGS warnings, $ISSUES issues"
    echo "📄 Detailed report saved to deployment_check_report_*.txt"
}

# Run main function
main "$@"
