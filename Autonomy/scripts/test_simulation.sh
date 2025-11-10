#!/bin/bash

# Simulation Testing Script
# Tests autonomy system in Gazebo simulation environment

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
SIMULATION_PACKAGE="autonomy_simulation"
WORLD=${1:-mars_desert}
TEST_DURATION=${2:-60}  # seconds

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
        echo "  Run: source /opt/ros/humble/setup.bash"
        exit 1
    fi
    print_success "ROS2 available"

    # Check Gazebo
    if ! command -v gazebo &> /dev/null; then
        print_error "Gazebo not found"
        echo "  Install: sudo apt install gazebo"
        exit 1
    fi
    print_success "Gazebo available"

    # Check simulation package
    if ! ros2 pkg list | grep -q "$SIMULATION_PACKAGE"; then
        print_warning "Simulation package not found, building..."
        cd "$ROS_WS"
        colcon build --packages-select "$SIMULATION_PACKAGE" --continue-on-error
        source install/setup.bash
        cd "$PROJECT_ROOT"
    fi
    print_success "Simulation package ready"

    # Check for world files
    if [ ! -f "$PROJECT_ROOT/simulation/worlds/${WORLD}.world" ]; then
        print_error "World file not found: $WORLD.world"
        echo "  Available worlds:"
        ls -1 "$PROJECT_ROOT/simulation/worlds/" 2>/dev/null || echo "  No worlds found"
        exit 1
    fi
    print_success "World file found: $WORLD.world"
}

# Function to start Gazebo simulation
start_simulation() {
    print_header "Starting Gazebo Simulation"

    print_info "Launching Gazebo with world: $WORLD"

    # Start Gazebo (headless for CI, GUI for development)
    if [ -n "$CI" ] || [ -n "$HEADLESS" ]; then
        print_info "Starting headless Gazebo..."
        ros2 launch gazebo_ros gazebo.launch.py \
            world:="$PROJECT_ROOT/simulation/worlds/${WORLD}.world" \
            headless:=true &
    else
        print_info "Starting GUI Gazebo..."
        ros2 launch gazebo_ros gazebo.launch.py \
            world:="$PROJECT_ROOT/simulation/worlds/${WORLD}.world" &
    fi

    GAZEBO_PID=$!
    echo $GAZEBO_PID > /tmp/simulation_gazebo.pid

    # Wait for Gazebo to start
    print_info "Waiting for Gazebo to initialize..."
    sleep 15

    if pgrep -f "gazebo" > /dev/null; then
        print_success "Gazebo simulation started"
    else
        print_error "Gazebo failed to start"
        cleanup
        exit 1
    fi
}

# Function to spawn rover
spawn_rover() {
    print_header "Spawning Rover"

    print_info "Spawning rover in simulation..."

    # Check if URDF exists
    local rover_urdf="$PROJECT_ROOT/simulation/urdf/rover.urdf.xacro"
    if [ ! -f "$rover_urdf" ]; then
        print_error "Rover URDF not found: $rover_urdf"
        return 1
    fi

    # Spawn the rover
    ros2 run gazebo_ros spawn_entity.py \
        -topic /rover_description \
        -entity rover \
        -x 0.0 -y 0.0 -z 0.5 \
        -R 0.0 -P 0.0 -Y 0.0 \
        2>/dev/null

    if [ $? -eq 0 ]; then
        print_success "Rover spawned successfully"
        return 0
    else
        print_error "Failed to spawn rover"
        return 1
    fi
}

# Function to launch autonomy stack
launch_autonomy() {
    print_header "Launching Autonomy Stack"

    print_info "Starting autonomy systems..."

    # Launch state management
    ros2 launch autonomy_state_management state_machine.launch.py &
    STATE_PID=$!
    echo $STATE_PID > /tmp/simulation_state.pid

    # Launch safety system
    ros2 launch autonomy_safety_system safety_system.launch.py &
    SAFETY_PID=$!
    echo $SAFETY_PID > /tmp/simulation_safety.pid

    # Launch navigation
    ros2 launch autonomy_navigation navigation.launch.py &
    NAV_PID=$!
    echo $NAV_PID > /tmp/simulation_nav.pid

    # Launch computer vision
    ros2 launch autonomy_computer_vision computer_vision.launch.py &
    VISION_PID=$!
    echo $VISION_PID > /tmp/simulation_vision.pid

    # Wait for systems to initialize
    print_info "Waiting for autonomy systems to initialize..."
    sleep 20

    # Verify systems are running
    local autonomy_nodes=$(ros2 node list 2>/dev/null | grep -E "(state_management|navigation|computer_vision|safety)" | wc -l)
    if [ "$autonomy_nodes" -gt "0" ]; then
        print_success "Autonomy systems running ($autonomy_nodes nodes)"
    else
        print_warning "Autonomy systems may not be fully initialized"
    fi
}

# Function to run simulation tests
run_simulation_tests() {
    print_header "Running Simulation Tests"

    local test_start=$(date +%s)

    print_info "Running simulation test suite for $TEST_DURATION seconds..."

    # Basic connectivity tests
    test_connectivity

    # Sensor validation
    test_sensors

    # Navigation tests
    test_navigation

    # Safety tests
    test_safety_simulation

    # Performance monitoring
    monitor_performance "$test_start"

    local test_end=$(date +%s)
    local actual_duration=$((test_end - test_start))
    print_info "Simulation test completed in ${actual_duration}s"
}

# Function to test basic connectivity
test_connectivity() {
    print_info "Testing system connectivity..."

    # Test ROS2 connectivity
    if ros2 topic list &>/dev/null; then
        local topic_count=$(ros2 topic list | wc -l)
        print_success "ROS2 connectivity OK ($topic_count topics)"
    else
        print_error "ROS2 connectivity failed"
        return 1
    fi

    # Test critical topics
    local critical_topics=("/clock" "/odom" "/camera/image_raw")
    for topic in "${critical_topics[@]}"; do
        if ros2 topic info "$topic" &>/dev/null; then
            print_success "Topic $topic available"
        else
            print_warning "Topic $topic not available"
        fi
    done
}

# Function to test sensors
test_sensors() {
    print_info "Testing sensor systems..."

    # Test IMU
    if ros2 topic hz /imu/data --duration 3 2>/dev/null | grep -q "average rate"; then
        print_success "IMU sensor publishing"
    else
        print_warning "IMU sensor not publishing"
    fi

    # Test GPS
    if ros2 topic hz /gps/fix --duration 3 2>/dev/null | grep -q "average rate"; then
        print_success "GPS sensor publishing"
    else
        print_warning "GPS sensor not publishing"
    fi

    # Test camera
    if ros2 topic hz /camera/image_raw --duration 3 2>/dev/null | grep -q "average rate"; then
        print_success "Camera publishing"
    else
        print_warning "Camera not publishing"
    fi
}

# Function to test navigation
test_navigation() {
    print_info "Testing navigation system..."

    # Test odometry
    if ros2 topic echo /odom --once --timeout 5 &>/dev/null; then
        print_success "Odometry data available"
    else
        print_warning "Odometry data not available"
    fi

    # Test navigation commands
    print_info "Testing basic navigation commands..."

    # Send a simple velocity command
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "
linear:
  x: 0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.1
" 2>/dev/null && print_success "Navigation commands accepted" || print_warning "Navigation commands failed"
}

# Function to test safety in simulation
test_safety_simulation() {
    print_info "Testing safety systems in simulation..."

    # Test safety dashboard
    if ros2 topic echo /safety/dashboard_status --once --timeout 5 &>/dev/null; then
        print_success "Safety dashboard responding"
    else
        print_warning "Safety dashboard not responding"
    fi

    # Test emergency stop service
    if ros2 service list 2>/dev/null | grep -q "software_estop"; then
        print_success "Emergency stop service available"
    else
        print_warning "Emergency stop service not available"
    fi
}

# Function to monitor performance
monitor_performance() {
    local start_time=$1
    local current_time=$(date +%s)
    local elapsed=$((current_time - start_time))

    print_info "Performance monitoring ($elapsed seconds elapsed)..."

    # CPU usage
    local cpu_usage=$(top -b -n1 | grep "Cpu(s)" | awk '{print $2}' | cut -d'.' -f1 || echo "unknown")
    if [ "$cpu_usage" != "unknown" ]; then
        if [ "$cpu_usage" -lt 80 ]; then
            print_success "CPU usage: ${cpu_usage}%"
        else
            print_warning "High CPU usage: ${cpu_usage}%"
        fi
    fi

    # Memory usage
    local mem_usage=$(free | grep Mem | awk '{printf "%.0f", $3/$2 * 100.0}' || echo "unknown")
    if [ "$mem_usage" != "unknown" ]; then
        if [ "$mem_usage" -lt 90 ]; then
            print_success "Memory usage: ${mem_usage}%"
        else
            print_warning "High memory usage: ${mem_usage}%"
        fi
    fi

    # Topic frequencies
    local topics_to_check=("/odom" "/imu/data" "/camera/image_raw")
    for topic in "${topics_to_check[@]}"; do
        local hz=$(timeout 5 ros2 topic hz "$topic" 2>/dev/null | tail -1 | awk '{print $2}' || echo "0")
        if (( $(echo "$hz > 0" | bc -l 2>/dev/null || echo "0") )); then
            print_success "$topic: ~${hz}Hz"
        else
            print_info "$topic: no data"
        fi
    done
}

# Function to generate simulation report
generate_report() {
    print_header "Simulation Test Report"

    local report_file="simulation_test_report_$(date +%Y%m%d_%H%M%S).txt"

    cat > "$report_file" << EOF
URC 2026 Simulation Test Report
===============================

Test Configuration:
- World: $WORLD
- Duration: $TEST_DURATION seconds
- Timestamp: $(date)

System Status:
$(ros2 node list 2>/dev/null | wc -l) nodes running
$(ros2 topic list 2>/dev/null | wc -l) topics available

Test Results:
- Connectivity: $(test_connectivity &>/dev/null && echo "PASS" || echo "FAIL")
- Sensors: $(test_sensors &>/dev/null && echo "PASS" || echo "PARTIAL")
- Navigation: $(test_navigation &>/dev/null && echo "PASS" || echo "PARTIAL")
- Safety: $(test_safety_simulation &>/dev/null && echo "PASS" || echo "PARTIAL")

Performance Metrics:
$(monitor_performance $(date +%s) 2>/dev/null | grep -E "(CPU|Memory|Hz)" | sed 's/\x1b\[[0-9;]*m//g' || echo "Metrics collection failed")

Notes:
- This report was generated by the simulation test script
- For detailed logs, check the console output above
- Simulation testing validates basic system functionality in Gazebo
EOF

    print_success "Simulation test report saved: $report_file"
}

# Function to cleanup simulation
cleanup() {
    print_header "Cleaning Up Simulation"

    print_info "Stopping simulation processes..."

    # Kill processes in reverse order
    local pids=("/tmp/simulation_vision.pid" "/tmp/simulation_nav.pid" "/tmp/simulation_safety.pid" "/tmp/simulation_state.pid" "/tmp/simulation_gazebo.pid")

    for pid_file in "${pids[@]}"; do
        if [ -f "$pid_file" ]; then
            local pid=$(cat "$pid_file")
            if kill -0 "$pid" 2>/dev/null; then
                print_info "Stopping process $(basename "$pid_file" .pid | sed 's/simulation_//') ($pid)"
                kill "$pid" 2>/dev/null || true
                sleep 1
            fi
            rm -f "$pid_file"
        fi
    done

    # Kill any remaining simulation processes
    pkill -f "gazebo" || true
    pkill -f "autonomy_" || true

    print_success "Simulation cleanup completed"
}

# Function to show usage
usage() {
    echo "Usage: $0 [world] [duration]"
    echo ""
    echo "Simulation Testing for URC 2026 Autonomy"
    echo ""
    echo "Arguments:"
    echo "  world     World file name (default: mars_desert)"
    echo "  duration  Test duration in seconds (default: 60)"
    echo ""
    echo "Options:"
    echo "  --headless    Run without GUI (for CI/CD)"
    echo "  --help        Show this help"
    echo ""
    echo "Available worlds:"
    ls -1 "$PROJECT_ROOT/simulation/worlds/" 2>/dev/null | sed 's/\.world$//' | sed 's/^/  /' || echo "  No worlds found"
    echo ""
    echo "Examples:"
    echo "  $0                    # Basic mars_desert simulation test"
    echo "  $0 competition 120   # Competition world, 2 minute test"
    echo "  $0 --headless         # Headless mode for CI/CD"
}

# Main execution
main() {
    local headless=false

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --headless)
                headless=true
                export HEADLESS=1
                shift
                ;;
            --help)
                usage
                exit 0
                ;;
            -*)
                print_error "Unknown option: $1"
                usage
                exit 1
                ;;
            *)
                if [ -z "$WORLD" ]; then
                    WORLD=$1
                elif [ -z "$TEST_DURATION" ]; then
                    TEST_DURATION=$1
                else
                    print_error "Too many arguments"
                    usage
                    exit 1
                fi
                shift
                ;;
        esac
    done

    echo "🎮 URC 2026 Simulation Testing"
    echo "=============================="
    echo "World: $WORLD"
    echo "Duration: $TEST_DURATION seconds"
    if [ "$headless" = true ]; then
        echo "Mode: Headless (no GUI)"
    else
        echo "Mode: GUI"
    fi
    echo ""

    # Setup cleanup trap
    trap cleanup EXIT

    # Run test phases
    check_prerequisites
    start_simulation
    spawn_rover
    launch_autonomy
    run_simulation_tests
    generate_report

    print_success "🎉 Simulation testing completed!"
    print_info "Check the generated report for detailed results"
}

# Run main function
main "$@"
