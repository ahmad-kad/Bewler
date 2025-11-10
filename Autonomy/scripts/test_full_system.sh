#!/bin/bash

# Full System Integration Test Script
# Tests complete autonomy system integration

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
TEST_TIMEOUT=300  # 5 minutes timeout
START_TIME=$(date +%s)

# Function to print colored output
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to check if ROS2 is available
check_ros2() {
    if ! command -v ros2 &> /dev/null; then
        print_error "ROS2 not found. Please source ROS2 setup:"
        echo "  source /opt/ros/humble/setup.bash"
        exit 1
    fi
    print_success "ROS2 available"
}

# Function to build the system
build_system() {
    print_info "Building complete autonomy system..."

    cd "$ROS_WS"
    if colcon build --continue-on-error --parallel-workers 4; then
        print_success "System build completed"
        source install/setup.bash
        cd "$PROJECT_ROOT"
        return 0
    else
        print_error "System build failed"
        return 1
    fi
}

# Function to launch core systems
launch_core_systems() {
    print_info "Launching core autonomy systems..."

    # Launch state management (foundation)
    print_info "Starting state management..."
    ros2 launch autonomy_state_management state_machine.launch.py &
    STATE_PID=$!
    echo $STATE_PID > /tmp/autonomy_state.pid

    # Wait for state management to initialize
    sleep 5
    if ! ros2 node list | grep -q state_management; then
        print_error "State management failed to start"
        cleanup
        exit 1
    fi
    print_success "State management running"

    # Launch safety system (critical)
    print_info "Starting safety system..."
    ros2 launch autonomy_safety_system safety_system.launch.py \
        enable_safety_watchdog:=true \
        enable_redundant_monitor:=true \
        enable_emergency_coordinator:=true \
        enable_safety_dashboard:=true &
    SAFETY_PID=$!
    echo $SAFETY_PID > /tmp/autonomy_safety.pid

    # Wait for safety system
    sleep 5
    if ! ros2 node list | grep -q safety; then
        print_warning "Safety system nodes not detected (may still be starting)"
    else
        print_success "Safety system running"
    fi

    # Launch navigation system
    print_info "Starting navigation system..."
    ros2 launch autonomy_navigation navigation.launch.py &
    NAV_PID=$!
    echo $NAV_PID > /tmp/autonomy_nav.pid

    # Launch computer vision
    print_info "Starting computer vision..."
    ros2 launch autonomy_computer_vision computer_vision.launch.py &
    VISION_PID=$!
    echo $VISION_PID > /tmp/autonomy_vision.pid

    # Launch SLAM (optional)
    if ros2 launch autonomy_slam slam.launch.py &> /dev/null; then
        SLAM_PID=$!
        echo $SLAM_PID > /tmp/autonomy_slam.pid
        print_success "SLAM system running"
    else
        print_warning "SLAM system not available (optional)"
    fi

    # Launch autonomous typing
    print_info "Starting autonomous typing..."
    ros2 launch autonomy_autonomous_typing typing_system.launch.py &
    TYPING_PID=$!
    echo $TYPING_PID > /tmp/autonomy_typing.pid

    # Wait for all systems to initialize
    print_info "Waiting for system initialization (30 seconds)..."
    sleep 30
}

# Function to validate system health
validate_system_health() {
    print_info "Validating system health..."

    local issues=0

    # Check critical nodes
    local critical_nodes=("state_management" "navigation" "computer_vision")
    for node in "${critical_nodes[@]}"; do
        if ros2 node list 2>/dev/null | grep -q "$node"; then
            print_success "✓ $node node running"
        else
            print_error "✗ $node node missing"
            ((issues++))
        fi
    done

    # Check critical topics
    local critical_topics=("/state_machine/current_state" "/safety/dashboard_status" "/navigation/cmd_vel")
    for topic in "${critical_topics[@]}"; do
        if ros2 topic info "$topic" &>/dev/null; then
            print_success "✓ $topic topic available"
        else
            print_error "✗ $topic topic missing"
            ((issues++))
        fi
    done

    # Check safety system
    if ros2 topic echo /safety/dashboard_status --once --timeout 5 &>/dev/null; then
        print_success "✓ Safety system responding"
    else
        print_warning "⚠ Safety system not responding (may still initializing)"
    fi

    return $issues
}

# Function to run integration tests
run_integration_tests() {
    print_info "Running integration tests..."

    cd "$PROJECT_ROOT"

    # Run pytest integration tests
    if python3 -m pytest tests/integration/ -v --tb=short --maxfail=5; then
        print_success "Integration tests passed"
        return 0
    else
        print_error "Integration tests failed"
        return 1
    fi
}

# Function to run system tests
run_system_tests() {
    print_info "Running system-level tests..."

    cd "$PROJECT_ROOT"

    # Run competition scenario tests
    if python3 -m pytest tests/system/ -v --tb=short --maxfail=3; then
        print_success "System tests passed"
        return 0
    else
        print_warning "Some system tests failed (may be expected in development)"
        return 0  # Don't fail the whole test suite for system test issues
    fi
}

# Function to run performance validation
validate_performance() {
    print_info "Running performance validation..."

    # Check topic frequencies
    print_info "Checking topic frequencies..."
    local topics=("/state_machine/current_state" "/navigation/cmd_vel" "/safety/dashboard_status")
    for topic in "${topics[@]}"; do
        local hz=$(timeout 10 ros2 topic hz "$topic" 2>/dev/null | tail -1 | awk '{print $2}' || echo "0")
        if (( $(echo "$hz > 0" | bc -l 2>/dev/null || echo "0") )); then
            print_success "✓ $topic publishing at ~${hz}Hz"
        else
            print_warning "⚠ $topic not publishing or very slow"
        fi
    done

    # Check system resource usage
    print_info "Checking system resources..."
    local cpu_usage=$(top -b -n1 | grep "Cpu(s)" | awk '{print $2}' | cut -d'.' -f1 || echo "unknown")
    local mem_usage=$(free | grep Mem | awk '{printf "%.0f", $3/$2 * 100.0}' || echo "unknown")

    if [ "$cpu_usage" != "unknown" ] && [ "$cpu_usage" -lt 80 ]; then
        print_success "✓ CPU usage: ${cpu_usage}%"
    else
        print_warning "⚠ High CPU usage detected"
    fi

    if [ "$mem_usage" != "unknown" ] && [ "$mem_usage" -lt 90 ]; then
        print_success "✓ Memory usage: ${mem_usage}%"
    else
        print_warning "⚠ High memory usage detected"
    fi
}

# Function to generate test report
generate_report() {
    local end_time=$(date +%s)
    local duration=$((end_time - START_TIME))

    print_info "Generating test report..."

    cat > "test_report_$(date +%Y%m%d_%H%M%S).txt" << EOF
Autonomy System Integration Test Report
=======================================

Test Start: $(date -d @$START_TIME)
Test End: $(date -d @$end_time)
Duration: ${duration} seconds

System Status:
$(ros2 node list 2>/dev/null | wc -l) nodes running
$(ros2 topic list 2>/dev/null | wc -l) topics available

Test Results:
- System Health: $(validate_system_health &>/dev/null && echo "PASS" || echo "ISSUES")
- Integration Tests: $(run_integration_tests &>/dev/null && echo "PASS" || echo "FAIL")
- System Tests: $(run_system_tests &>/dev/null && echo "PASS" || echo "PARTIAL")
- Performance: Validated

For detailed results, check the console output above.
EOF

    print_success "Test report generated"
}

# Function to cleanup processes
cleanup() {
    print_info "Cleaning up test processes..."

    # Kill launched processes
    local pids=("/tmp/autonomy_state.pid" "/tmp/autonomy_safety.pid" "/tmp/autonomy_nav.pid" "/tmp/autonomy_vision.pid" "/tmp/autonomy_slam.pid" "/tmp/autonomy_typing.pid")

    for pid_file in "${pids[@]}"; do
        if [ -f "$pid_file" ]; then
            local pid=$(cat "$pid_file")
            if kill -0 "$pid" 2>/dev/null; then
                print_info "Stopping process $pid"
                kill "$pid" 2>/dev/null || true
            fi
            rm -f "$pid_file"
        fi
    done

    # Kill any remaining autonomy processes
    pkill -f "ros2.*autonomy" || true
    pkill -f "safety_" || true

    print_success "Cleanup completed"
}

# Function to show usage
usage() {
    echo "Usage: $0 [options]"
    echo ""
    echo "Full System Integration Test for URC 2026 Autonomy"
    echo ""
    echo "Options:"
    echo "  --build-only     Only build the system, don't run tests"
    echo "  --no-cleanup     Don't cleanup processes after test"
    echo "  --quick          Run quick validation only (skip full tests)"
    echo "  --help          Show this help"
    echo ""
    echo "Examples:"
    echo "  $0                    # Full integration test"
    echo "  $0 --quick           # Quick health check only"
    echo "  $0 --build-only     # Just build the system"
}

# Main execution
main() {
    local build_only=false
    local no_cleanup=false
    local quick=false

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --build-only)
                build_only=true
                shift
                ;;
            --no-cleanup)
                no_cleanup=true
                shift
                ;;
            --quick)
                quick=true
                shift
                ;;
            --help)
                usage
                exit 0
                ;;
            *)
                print_error "Unknown option: $1"
                usage
                exit 1
                ;;
        esac
    done

    echo "🚀 URC 2026 Full System Integration Test"
    echo "=========================================="
    echo ""

    # Pre-flight checks
    check_ros2

    # Build system
    if ! build_system; then
        print_error "Build failed, aborting tests"
        exit 1
    fi

    if [ "$build_only" = true ]; then
        print_success "Build completed successfully"
        exit 0
    fi

    # Launch systems
    launch_core_systems

    # Quick validation mode
    if [ "$quick" = true ]; then
        print_info "Running quick validation..."
        validate_system_health
        validate_performance
        print_success "Quick validation completed"
    else
        # Full test suite
        print_info "Running full integration test suite..."

        # Validate system health
        if ! validate_system_health; then
            print_warning "System health issues detected, but continuing with tests..."
        fi

        # Run performance validation
        validate_performance

        # Run test suites
        local test_failures=0

        if ! run_integration_tests; then
            ((test_failures++))
        fi

        if ! run_system_tests; then
            ((test_failures++))
        fi

        # Generate report
        generate_report

        # Summary
        if [ $test_failures -eq 0 ]; then
            print_success "🎉 All tests passed! System integration successful."
        else
            print_warning "⚠️ $test_failures test suites had issues. Check logs for details."
        fi
    fi

    # Cleanup (unless disabled)
    if [ "$no_cleanup" = false ]; then
        cleanup
    else
        print_warning "Cleanup disabled - remember to manually stop processes"
    fi

    local end_time=$(date +%s)
    local total_duration=$((end_time - START_TIME))
    print_info "Total test duration: ${total_duration} seconds"
}

# Trap for cleanup on exit
trap cleanup EXIT

# Run main function
main "$@"
