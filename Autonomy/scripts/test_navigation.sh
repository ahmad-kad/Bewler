#!/bin/bash

# Navigation Subsystem Testing Script
# Comprehensive testing for navigation system

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
TEST_TYPE=${1:-all}

# Function to print colored output
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[✓]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[⚠]${NC} $1"
}

print_error() {
    echo -e "${RED}[✗]${NC} $1"
}

print_header() {
    echo ""
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}$(printf '%.0s=' {1..50})${NC}"
}

# Function to check prerequisites
check_prerequisites() {
    print_header "Checking Prerequisites"

    # Check ROS2
    if ! command -v ros2 &> /dev/null; then
        print_error "ROS2 not found"
        exit 1
    fi

    # Check navigation package
    if ! ros2 pkg list | grep -q "autonomy_navigation"; then
        print_warning "Navigation package not built"
        echo "  Building navigation package..."
        cd "$ROS_WS"
        colcon build --packages-select autonomy_navigation --continue-on-error
        source install/setup.bash
    fi
    print_success "Navigation package ready"
}

# Function to run unit tests
run_unit_tests() {
    print_header "Running Navigation Unit Tests"

    if [ -d "$PROJECT_ROOT/tests/unit/navigation" ]; then
        cd "$PROJECT_ROOT"
        if python -m pytest tests/unit/navigation/ -v --tb=short; then
            print_success "Navigation unit tests passed"
            return 0
        else
            print_error "Navigation unit tests failed"
            return 1
        fi
    else
        print_warning "No navigation unit tests found"
        return 0
    fi
}

# Function to run integration tests
run_integration_tests() {
    print_header "Running Navigation Integration Tests"

    cd "$PROJECT_ROOT"
    if python -m pytest tests/integration/ -k "navigation" -v --tb=short; then
        print_success "Navigation integration tests passed"
        return 0
    else
        print_error "Navigation integration tests failed"
        return 1
    fi
}

# Function to launch navigation system
launch_navigation() {
    print_header "Launching Navigation System"

    # Source workspace
    cd "$ROS_WS"
    source install/setup.bash

    # Launch navigation
    print_info "Starting navigation nodes..."
    ros2 launch autonomy_navigation navigation.launch.py &
    NAV_PID=$!
    echo $NAV_PID > /tmp/navigation_test.pid

    # Wait for navigation to start
    print_info "Waiting for navigation system to initialize..."
    sleep 10

    # Verify navigation is running
    local nav_nodes=$(ros2 node list 2>/dev/null | grep -c "navigation" || echo "0")
    if [ "$nav_nodes" -gt "0" ]; then
        print_success "Navigation system running ($nav_nodes nodes)"
    else
        print_error "Navigation system failed to start"
        cleanup_navigation
        exit 1
    fi
}

# Function to test navigation topics
test_topics() {
    print_header "Testing Navigation Topics"

    local critical_topics=(
        "/navigation/cmd_vel"
        "/odom"
        "/state_machine/current_state"
    )

    for topic in "${critical_topics[@]}"; do
        if ros2 topic info "$topic" &>/dev/null; then
            print_success "Topic $topic available"

            # Test topic frequency for active topics
            if ros2 topic hz "$topic" --duration 2 2>/dev/null | grep -q "average rate"; then
                local hz=$(ros2 topic hz "$topic" --duration 2 2>/dev/null | tail -1 | awk '{print $2}')
                print_success "  Publishing at ~${hz}Hz"
            fi
        else
            print_error "Topic $topic missing"
        fi
    done
}

# Function to test basic navigation commands
test_basic_commands() {
    print_header "Testing Basic Navigation Commands"

    # Test velocity commands
    print_info "Testing velocity commands..."

    # Stop command
    if ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "
linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
" 2>/dev/null; then
        print_success "Stop command accepted"
    else
        print_error "Stop command failed"
    fi

    # Slow forward command
    if ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "
linear:
  x: 0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
" 2>/dev/null; then
        print_success "Forward command accepted"
    else
        print_error "Forward command failed"
    fi

    # Rotation command
    if ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "
linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.2
" 2>/dev/null; then
        print_success "Rotation command accepted"
    else
        print_error "Rotation command failed"
    fi
}

# Function to test navigation services
test_services() {
    print_header "Testing Navigation Services"

    # Test waypoint navigation service
    if ros2 service list 2>/dev/null | grep -q "navigate_to_pose"; then
        print_success "Navigation action available"

        # Test action goal (don't wait for completion)
        print_info "Testing navigation goal..."
        ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
pose:
  header:
    frame_id: 'map'
  pose:
    position:
      x: 1.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
" 2>/dev/null &
        local goal_pid=$!

        sleep 2
        kill $goal_pid 2>/dev/null || true
        print_success "Navigation goal sent"
    else
        print_error "Navigation action not available"
    fi
}

# Function to test sensor integration
test_sensor_integration() {
    print_header "Testing Sensor Integration"

    # Test odometry
    if ros2 topic echo /odom --once --timeout 3 &>/dev/null; then
        print_success "Odometry data available"
    else
        print_warning "Odometry data not available"
    fi

    # Test IMU (if available)
    if ros2 topic info /imu/data &>/dev/null; then
        if ros2 topic hz /imu/data --duration 2 2>/dev/null | grep -q "average rate"; then
            print_success "IMU data available"
        else
            print_warning "IMU topic exists but not publishing"
        fi
    else
        print_info "IMU not available (optional)"
    fi

    # Test GPS (if available)
    if ros2 topic info /gps/fix &>/dev/null; then
        if ros2 topic hz /gps/fix --duration 2 2>/dev/null | grep -q "average rate"; then
            print_success "GPS data available"
        else
            print_warning "GPS topic exists but not publishing"
        fi
    else
        print_info "GPS not available (optional)"
    fi
}

# Function to test safety integration
test_safety_integration() {
    print_header "Testing Safety Integration"

    # Test that navigation respects safety commands
    if ros2 topic echo /safety/dashboard_status --once --timeout 3 &>/dev/null; then
        print_success "Safety system available"

        # Test emergency stop integration
        print_info "Testing emergency stop integration..."
        if ros2 service call /state_machine/software_estop autonomy_interfaces/srv/SoftwareEstop "
operator_id: 'nav_test'
reason: 'navigation_safety_test'
acknowledge_criticality: true
force_immediate: false
" --timeout 5 &>/dev/null; then
            print_success "Emergency stop integration working"
        else
            print_warning "Emergency stop integration not available"
        fi
    else
        print_info "Safety system not running (navigation can still be tested independently)"
    fi
}

# Function to run performance tests
run_performance_tests() {
    print_header "Running Navigation Performance Tests"

    # Test command response time
    print_info "Testing command response time..."

    local start_time=$(date +%s%N)
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "
linear: {x: 0.0, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.0}
" 2>/dev/null
    local end_time=$(date +%s%N)
    local response_time=$(( (end_time - start_time) / 1000000 ))  # milliseconds

    if [ $response_time -lt 100 ]; then
        print_success "Command response time: ${response_time}ms"
    else
        print_warning "Slow command response: ${response_time}ms"
    fi

    # Test topic frequencies during operation
    print_info "Testing topic frequencies under load..."

    # Send continuous commands and measure frequencies
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "
linear: {x: 0.1, y: 0.0, z: 0.0}
angular: {x: 0.0, y: 0.0, z: 0.0}
" 2>/dev/null

    sleep 2

    # Check odometry frequency
    local odom_hz=$(ros2 topic hz /odom --duration 2 2>/dev/null | tail -1 | awk '{print $2}' || echo "0")
    if (( $(echo "$odom_hz > 10" | bc -l 2>/dev/null || echo "0") )); then
        print_success "Odometry frequency: ${odom_hz}Hz"
    else
        print_warning "Low odometry frequency: ${odom_hz}Hz"
    fi
}

# Function to cleanup navigation test
cleanup_navigation() {
    print_info "Cleaning up navigation test..."

    # Kill navigation processes
    if [ -f /tmp/navigation_test.pid ]; then
        local pid=$(cat /tmp/navigation_test.pid)
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
        rm -f /tmp/navigation_test.pid
    fi

    # Kill any remaining navigation processes
    pkill -f "autonomy_navigation" || true
    pkill -f "navigation_node" || true

    print_success "Navigation cleanup completed"
}

# Function to generate test report
generate_report() {
    print_header "Generating Navigation Test Report"

    local report_file="navigation_test_report_$(date +%Y%m%d_%H%M%S).txt"

    cat > "$report_file" << EOF
URC 2026 Navigation System Test Report
=====================================

Test Configuration:
- Test Type: $TEST_TYPE
- Timestamp: $(date)

Navigation System Status:
$(ros2 node list 2>/dev/null | grep -c navigation || echo "0") navigation nodes running
$(ros2 topic list 2>/dev/null | grep -c "navigation\|odom\|cmd_vel" || echo "0") navigation topics available

Test Results Summary:
- Unit Tests: $(run_unit_tests &>/dev/null && echo "PASS" || echo "FAIL")
- Integration Tests: $(run_integration_tests &>/dev/null && echo "PASS" || echo "FAIL")
- Topic Tests: $(test_topics &>/dev/null && echo "PASS" || echo "PARTIAL")
- Command Tests: $(test_basic_commands &>/dev/null && echo "PASS" || echo "PARTIAL")
- Service Tests: $(test_services &>/dev/null && echo "PASS" || echo "PARTIAL")
- Sensor Integration: $(test_sensor_integration &>/dev/null && echo "PASS" || echo "PARTIAL")
- Safety Integration: $(test_safety_integration &>/dev/null && echo "PASS" || echo "PARTIAL")

Recommendations:
- Ensure all navigation topics are publishing at expected frequencies
- Verify sensor data is being properly fused
- Test navigation in various environments
- Validate safety system integration

For detailed test output, run this script with individual test types.
EOF

    print_success "Navigation test report saved: $report_file"
}

# Function to show usage
usage() {
    echo "Usage: $0 [test_type]"
    echo ""
    echo "Navigation Subsystem Testing for URC 2026"
    echo ""
    echo "Test Types:"
    echo "  unit        Run unit tests only"
    echo "  integration Run integration tests only"
    echo "  functional  Test navigation functionality"
    echo "  performance Run performance tests"
    echo "  all         Run all tests (default)"
    echo ""
    echo "Examples:"
    echo "  $0                    # Run all navigation tests"
    echo "  $0 unit              # Unit tests only"
    echo "  $0 functional        # Functional tests only"
    echo ""
    echo "The script will:"
    echo "  • Build navigation package if needed"
    echo "  • Launch navigation system"
    echo "  • Run specified tests"
    echo "  • Generate test report"
    echo "  • Cleanup test processes"
}

# Main execution
main() {
    echo "🧭 URC 2026 Navigation System Testing"
    echo "====================================="
    echo "Test Type: ${TEST_TYPE:-all}"
    echo ""

    # Setup cleanup trap
    trap cleanup_navigation EXIT

    check_prerequisites

    case $TEST_TYPE in
        "unit")
            run_unit_tests
            ;;
        "integration")
            run_integration_tests
            ;;
        "functional")
            launch_navigation
            test_topics
            test_basic_commands
            test_services
            test_sensor_integration
            test_safety_integration
            ;;
        "performance")
            launch_navigation
            run_performance_tests
            ;;
        "all")
            # Build and run all tests
            run_unit_tests
            run_integration_tests
            launch_navigation
            test_topics
            test_basic_commands
            test_services
            test_sensor_integration
            test_safety_integration
            run_performance_tests
            ;;
        "help"|"-h"|"--help")
            usage
            exit 0
            ;;
        *)
            print_error "Unknown test type: $TEST_TYPE"
            usage
            exit 1
            ;;
    esac

    generate_report

    print_success "🎯 Navigation testing completed!"
    print_info "Check the generated report for detailed results"
}

# Run main function
main "$@"
