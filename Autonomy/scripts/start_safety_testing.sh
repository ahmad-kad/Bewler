#!/bin/bash

# Safety Testing Startup Script
# Provides one-command safety system testing setup

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
WEB_PORT=5173
ROS_BRIDGE_PORT=9090

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

# Function to check ROS2 environment
check_ros2() {
    if ! command -v ros2 &> /dev/null; then
        print_error "ROS2 not found. Please source ROS2 setup first:"
        echo "  source /opt/ros/humble/setup.bash"
        exit 1
    fi
    print_success "ROS2 available"
}

# Function to build safety system
build_safety() {
    print_info "Building safety system..."

    cd "$ROS_WS"
    if colcon build --packages-select autonomy_safety_system autonomy_state_management autonomy_interfaces --continue-on-error; then
        print_success "Safety system build completed"
        source install/setup.bash
        cd "$PROJECT_ROOT"
        return 0
    else
        print_error "Safety system build failed"
        return 1
    fi
}

# Function to start ROS2 bridge
start_rosbridge() {
    print_info "Starting ROS2 WebSocket bridge..."

    # Check if rosbridge is already running
    if pgrep -f "rosbridge" > /dev/null; then
        print_warning "ROS2 bridge already running"
        return 0
    fi

    # Start rosbridge
    ros2 run rosbridge_server rosbridge_websocket &
    ROSBRIDGE_PID=$!
    echo $ROSBRIDGE_PID > /tmp/safety_rosbridge.pid

    # Wait for bridge to start
    sleep 3
    if pgrep -f "rosbridge" > /dev/null; then
        print_success "ROS2 bridge started on port $ROS_BRIDGE_PORT"
    else
        print_error "ROS2 bridge failed to start"
        return 1
    fi
}

# Function to launch safety system
launch_safety_system() {
    print_info "Launching safety system..."

    # Launch state machine first (dependency)
    print_info "Starting state management..."
    ros2 launch autonomy_state_management state_machine.launch.py &
    STATE_PID=$!
    echo $STATE_PID > /tmp/safety_state.pid

    # Wait for state machine
    sleep 5
    if ! ros2 node list 2>/dev/null | grep -q state_management; then
        print_error "State management failed to start"
        cleanup
        exit 1
    fi
    print_success "State management running"

    # Launch safety system
    print_info "Starting safety system..."
    ros2 launch autonomy_safety_system safety_system.launch.py \
        enable_safety_watchdog:=true \
        enable_redundant_monitor:=true \
        enable_emergency_coordinator:=true \
        enable_safety_dashboard:=true \
        enable_integration_tester:=true \
        log_level:=info &
    SAFETY_PID=$!
    echo $SAFETY_PID > /tmp/safety_system.pid

    # Wait for safety system to initialize
    print_info "Waiting for safety system initialization..."
    sleep 10

    # Verify safety system is running
    local safety_nodes=$(ros2 node list 2>/dev/null | grep -c safety || echo "0")
    if [ "$safety_nodes" -gt "0" ]; then
        print_success "Safety system running ($safety_nodes nodes)"
    else
        print_warning "Safety system nodes not detected (may still starting)"
    fi

    # Check critical safety topics
    local safety_topics=$(ros2 topic list 2>/dev/null | grep -c "/safety/" || echo "0")
    if [ "$safety_topics" -gt "0" ]; then
        print_success "Safety topics available ($safety_topics topics)"
    else
        print_warning "Safety topics not detected"
    fi
}

# Function to start frontend
start_frontend() {
    print_info "Starting safety testing frontend..."

    # Check if port is already in use
    if lsof -Pi :$WEB_PORT -sTCP:LISTEN -t >/dev/null; then
        print_warning "Port $WEB_PORT already in use (frontend may already be running)"
        return 0
    fi

    cd "$PROJECT_ROOT/frontend"

    # Check if node_modules exists
    if [ ! -d "node_modules" ]; then
        print_info "Installing frontend dependencies..."
        npm install
    fi

    # Start development server
    npm run dev &
    FRONTEND_PID=$!
    echo $FRONTEND_PID > /tmp/safety_frontend.pid

    # Wait for frontend to start
    sleep 5
    if lsof -Pi :$WEB_PORT -sTCP:LISTEN -t >/dev/null; then
        print_success "Frontend started on http://localhost:$WEB_PORT"
    else
        print_error "Frontend failed to start on port $WEB_PORT"
        return 1
    fi

    cd "$PROJECT_ROOT"
}

# Function to run initial safety tests
run_initial_tests() {
    print_info "Running initial safety validation..."

    # Wait a bit more for system to be fully ready
    sleep 5

    # Test safety dashboard connectivity
    if ros2 topic echo /safety/dashboard_status --once --timeout 5 &>/dev/null; then
        print_success "Safety dashboard responding"
    else
        print_warning "Safety dashboard not responding (may still initializing)"
    fi

    # Test emergency stop service availability
    if ros2 service list 2>/dev/null | grep -q "software_estop"; then
        print_success "Emergency stop service available"
    else
        print_error "Emergency stop service not available"
    fi

    # Test safety topic publishing
    local safety_topic_count=$(ros2 topic list 2>/dev/null | grep -c "/safety/" || echo "0")
    print_info "Safety topics detected: $safety_topic_count"
}

# Function to show status and access information
show_status() {
    echo ""
    print_success "🎯 Safety Testing Environment Ready!"
    echo ""
    echo "Active Services:"
    if [ -f /tmp/safety_rosbridge.pid ]; then
        echo -e "  • ROS2 Bridge:    http://localhost:$ROS_BRIDGE_PORT"
    fi
    if [ -f /tmp/safety_frontend.pid ]; then
        echo -e "  • Frontend:       http://localhost:$WEB_PORT"
    fi
    if [ -f /tmp/safety_state.pid ]; then
        echo -e "  • State Manager:  $(cat /tmp/safety_state.pid)"
    fi
    if [ -f /tmp/safety_system.pid ]; then
        echo -e "  • Safety System:  $(cat /tmp/safety_system.pid)"
    fi

    echo ""
    echo "Safety Testing Commands:"
    echo "  • Web Interface: Open http://localhost:$WEB_PORT and go to '🛡️ Safety Testing' tab"
    echo "  • Monitor Safety: ros2 topic echo /safety/dashboard_status"
    echo "  • Emergency Stop: ros2 service call /state_machine/software_estop autonomy_interfaces/srv/SoftwareEstop \"{operator_id: 'test', reason: 'manual_test', acknowledge_criticality: true}\""
    echo "  • Run Safety Tests: ros2 run autonomy_safety_system safety_integration_tester --scenario BATTERY_CRITICAL"
    echo "  • Check Health: ./scripts/health_check.sh"

    echo ""
    echo "Shutdown Commands:"
    echo "  • Stop All: ./scripts/stop_safety_testing.sh"
    echo "  • Quick Stop: pkill -f 'safety_' && pkill -f 'rosbridge' && pkill -f 'vite'"

    echo ""
    print_info "Press Ctrl+C to stop all services"
}

# Function to cleanup processes
cleanup() {
    print_info "Cleaning up safety testing processes..."

    # Kill launched processes in reverse order
    local pid_files=("/tmp/safety_frontend.pid" "/tmp/safety_system.pid" "/tmp/safety_state.pid" "/tmp/safety_rosbridge.pid")

    for pid_file in "${pid_files[@]}"; do
        if [ -f "$pid_file" ]; then
            local pid=$(cat "$pid_file")
            if kill -0 "$pid" 2>/dev/null; then
                print_info "Stopping process $pid"
                kill "$pid" 2>/dev/null || true
                sleep 1
                # Force kill if still running
                if kill -0 "$pid" 2>/dev/null; then
                    kill -9 "$pid" 2>/dev/null || true
                fi
            fi
            rm -f "$pid_file"
        fi
    done

    # Kill any remaining safety-related processes
    pkill -f "autonomy_safety_system" || true
    pkill -f "safety_" || true
    pkill -f "rosbridge" || true

    print_success "Cleanup completed"
}

# Function to show usage
usage() {
    echo "Usage: $0 [command]"
    echo ""
    echo "Safety Testing Environment Setup for URC 2026"
    echo ""
    echo "Commands:"
    echo "  build     - Build safety system only"
    echo "  launch    - Launch safety system only"
    echo "  frontend  - Start frontend only"
    echo "  test      - Run initial safety tests"
    echo "  status    - Show current status"
    echo "  all       - Build, launch, and start everything (default)"
    echo "  help      - Show this help"
    echo ""
    echo "Examples:"
    echo "  $0              # Full safety testing setup"
    echo "  $0 launch      # Just launch safety system"
    echo "  $0 status      # Check what's running"
    echo ""
    echo "The script will start:"
    echo "  • ROS2 WebSocket bridge on port $ROS_BRIDGE_PORT"
    echo "  • State management system"
    echo "  • Complete safety system"
    echo "  • Safety testing frontend on http://localhost:$WEB_PORT"
}

# Main execution
main() {
    local command=${1:-all}

    echo "🛡️ URC 2026 Safety Testing Environment"
    echo "======================================"

    case $command in
        "build")
            check_ros2
            build_safety
            ;;
        "launch")
            check_ros2
            launch_safety_system
            run_initial_tests
            ;;
        "frontend")
            start_frontend
            ;;
        "test")
            check_ros2
            run_initial_tests
            ;;
        "status")
            show_status
            ;;
        "all")
            check_ros2
            build_safety
            start_rosbridge
            launch_safety_system
            start_frontend
            run_initial_tests
            show_status

            # Wait for user interrupt
            print_info "Safety testing environment is running..."
            print_info "Press Ctrl+C to stop all services"

            # Trap for cleanup
            trap cleanup EXIT
            wait
            ;;
        "help"|"-h"|"--help")
            usage
            exit 0
            ;;
        *)
            print_error "Unknown command: $command"
            usage
            exit 1
            ;;
    esac
}

# Trap for cleanup on exit (only for 'all' command)
if [ "${1:-all}" = "all" ]; then
    trap cleanup EXIT
fi

# Run main function
main "$@"
