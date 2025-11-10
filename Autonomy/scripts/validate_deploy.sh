#!/bin/bash

# Deployment Validation
# 5-minute comprehensive deployment readiness check

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

ISSUES=0
WARNINGS=0
PASSED=0

print_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
print_success() { echo -e "${GREEN}[✓]${NC} $1"; ((PASSED++)) }
print_warning() { echo -e "${YELLOW}[⚠]${NC} $1"; ((WARNINGS++)) }
print_error() { echo -e "${RED}[✗]${NC} $1"; ((ISSUES++)) }

echo "🚀 Deployment Validation (5 min)"
echo "=================================="

# Check 1: System Environment
print_info "System environment..."
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    print_success "Linux system detected"
else
    print_error "Unsupported OS: $OSTYPE"
fi

# Check memory
local total_mem=$(free -g | grep "^Mem:" | awk '{print $2}')
if [ "$total_mem" -ge 8 ]; then
    print_success "Sufficient memory: ${total_mem}GB"
else
    print_warning "Low memory: ${total_mem}GB (8GB recommended)"
fi

# Check 2: ROS2 Workspace
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
ROS_WS="$PROJECT_ROOT/ros2_ws"

if [ -d "$ROS_WS/install" ]; then
    print_success "ROS2 workspace built"
else
    print_error "ROS2 workspace not built"
fi

# Check packages
local autonomy_pkgs=$(ros2 pkg list 2>/dev/null | grep -c autonomy || echo "0")
if [ "$autonomy_pkgs" -ge 3 ]; then
    print_success "Autonomy packages available: $autonomy_pkgs"
else
    print_error "Missing autonomy packages: $autonomy_pkgs found"
fi

# Check 3: Hardware Interfaces
print_info "Hardware interfaces..."
if [ -e "/dev/ttyACM0" ] || [ -e "/dev/ttyUSB0" ]; then
    print_success "Serial devices available"
else
    print_info "No serial devices (may be normal)"
fi

if lsusb 2>/dev/null | grep -q .; then
    print_success "USB devices detected"
else
    print_warning "No USB devices detected"
fi

# Check 4: Launch Files
print_info "Launch configurations..."
launch_files=(
    "autonomy_state_management/state_machine.launch.py"
    "autonomy_safety_system/safety_system.launch.py"
    "autonomy_navigation/navigation.launch.py"
)

for launch_file in "${launch_files[@]}"; do
    if [ -f "$ROS_WS/install/share/$launch_file" ]; then
        print_success "$launch_file exists"
    else
        print_error "$launch_file missing"
    fi
done

# Check 5: Configuration Files
print_info "Configuration files..."
config_dirs=("autonomy_safety_system/config" "autonomy_navigation/config")
for config_dir in "${config_dirs[@]}"; do
    if [ -d "$ROS_WS/install/share/$config_dir" ]; then
        local yaml_count=$(find "$ROS_WS/install/share/$config_dir" -name "*.yaml" 2>/dev/null | wc -l)
        if [ "$yaml_count" -gt 0 ]; then
            print_success "$config_dir: $yaml_count config files"
        else
            print_warning "$config_dir: no YAML files"
        fi
    else
        print_error "$config_dir missing"
    fi
done

# Check 6: System Launch Test
print_info "Testing system launch..."
cd "$ROS_WS"
source install/setup.bash

# Quick launch test
timeout 30 ros2 launch autonomy_state_management state_machine.launch.py &
LAUNCH_PID=$!
sleep 5

if ros2 node list 2>/dev/null | grep -q state_management; then
    print_success "System can launch successfully"
else
    print_error "System launch failed"
fi

# Cleanup
kill $LAUNCH_PID 2>/dev/null || true
sleep 2

# Check 7: URC 2026 Compliance
print_info "URC 2026 compliance..."
if ros2 pkg list 2>/dev/null | grep -q "autonomy_safety_system"; then
    print_success "Safety system for competition compliance"
else
    print_error "Safety system missing (required for URC)"
fi

# Summary
echo ""
echo "📊 Deployment Validation Results:"
echo "  Passed: $PASSED"
echo "  Warnings: $WARNINGS"
echo "  Critical Issues: $ISSUES"

echo ""
if [ $ISSUES -eq 0 ]; then
    print_success "✅ System ready for deployment!"
    if [ $WARNINGS -gt 0 ]; then
        echo "Address $WARNINGS warnings before competition."
    fi
else
    print_error "❌ $ISSUES critical issues prevent deployment"
    echo ""
    echo "Critical issues must be resolved:"
    echo "  • Build ROS2 workspace: cd ros2_ws && colcon build"
    echo "  • Install missing packages"
    echo "  • Configure hardware interfaces"
    echo "  • Test system launch"
fi

echo ""
echo "⏱️ Validation completed in ~3-5 minutes"
echo ""
echo "For competition readiness:"
echo "  ./scripts/validate_compete.sh  # Full competition validation"
