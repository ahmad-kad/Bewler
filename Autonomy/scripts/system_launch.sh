#!/bin/bash

# System Launch Script
# Simple, reliable system startup for autonomy testing

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Config
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
ROS_WS="$PROJECT_ROOT/ros2_ws"

print_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
print_success() { echo -e "${GREEN}[✓]${NC} $1"; }
print_warning() { echo -e "${YELLOW}[⚠]${NC} $1"; }
print_error() { echo -e "${RED}[✗]${NC} $1"; }

cleanup() {
    print_info "Cleaning up any existing processes..."
    pkill -f "autonomy_" || true
    pkill -f "ros2.*launch" || true
    sleep 2
}

check_prerequisites() {
    if ! command -v ros2 &> /dev/null; then
        print_error "ROS2 not found. Run: source /opt/ros/humble/setup.bash"
        exit 1
    fi

    if [ ! -f "$ROS_WS/install/setup.bash" ]; then
        print_error "ROS2 workspace not built. Run: cd ros2_ws && colcon build"
        exit 1
    fi

    print_success "Prerequisites OK"
}

launch_state_management() {
    print_info "Launching state management..."
    cd "$ROS_WS"
    source install/setup.bash
    ros2 launch autonomy_state_management state_machine.launch.py &
    echo $! > /tmp/system_state.pid
    sleep 3

    if ros2 node list 2>/dev/null | grep -q state_management; then
        print_success "State management running"
    else
        print_error "State management failed to start"
        exit 1
    fi
}

launch_safety_system() {
    print_info "Launching safety system..."
    ros2 launch autonomy_safety_system safety_system.launch.py &
    echo $! > /tmp/system_safety.pid
    sleep 5

    if ros2 node list 2>/dev/null | grep -q safety; then
        print_success "Safety system running"
    else
        print_warning "Safety system may still initializing"
    fi
}

launch_navigation() {
    print_info "Launching navigation..."
    ros2 launch autonomy_navigation navigation.launch.py &
    echo $! > /tmp/system_nav.pid
    sleep 2
    print_success "Navigation launched"
}

launch_vision() {
    print_info "Launching computer vision..."
    ros2 launch autonomy_computer_vision computer_vision.launch.py &
    echo $! > /tmp/system_vision.pid
    sleep 2
    print_success "Vision launched"
}

launch_optional_subsystems() {
    # Try to launch optional subsystems but don't fail if they're not available
    print_info "Launching optional subsystems..."

    # SLAM
    if ros2 launch autonomy_slam slam.launch.py &>/dev/null; then
        echo $! > /tmp/system_slam.pid
        print_success "SLAM launched"
    fi

    # Autonomous typing
    if ros2 launch autonomy_autonomous_typing typing_system.launch.py &>/dev/null; then
        echo $! > /tmp/system_typing.pid
        print_success "Autonomous typing launched"
    fi
}

wait_for_system() {
    print_info "Waiting for system stabilization..."
    sleep 10

    local nodes=$(ros2 node list 2>/dev/null | wc -l)
    local topics=$(ros2 topic list 2>/dev/null | wc -l)

    print_success "System running: $nodes nodes, $topics topics"
}

show_status() {
    echo ""
    print_success "🚀 Autonomy System Launched!"
    echo ""
    echo "Running components:"
    [ -f /tmp/system_state.pid ] && echo "  • State Management (PID: $(cat /tmp/system_state.pid))"
    [ -f /tmp/system_safety.pid ] && echo "  • Safety System (PID: $(cat /tmp/system_safety.pid))"
    [ -f /tmp/system_nav.pid ] && echo "  • Navigation (PID: $(cat /tmp/system_nav.pid))"
    [ -f /tmp/system_vision.pid ] && echo "  • Computer Vision (PID: $(cat /tmp/system_vision.pid))"
    [ -f /tmp/system_slam.pid ] && echo "  • SLAM (PID: $(cat /tmp/system_slam.pid))"
    [ -f /tmp/system_typing.pid ] && echo "  • Autonomous Typing (PID: $(cat /tmp/system_typing.pid))"
    echo ""
    echo "Test the system:"
    echo "  ./scripts/system_test.sh      # Run integration tests"
    echo "  ./scripts/health_check.sh     # Quick health validation"
    echo ""
    echo "Stop the system:"
    echo "  ./scripts/system_stop.sh      # Clean shutdown"
}

usage() {
    echo "Usage: $0 [options]"
    echo ""
    echo "Launch URC 2026 Autonomy System"
    echo ""
    echo "Options:"
    echo "  --minimal    Launch only core systems (state + safety)"
    echo "  --full       Launch all systems including optional (default)"
    echo "  --help       Show this help"
    echo ""
    echo "Core systems: State Management, Safety System"
    echo "Main systems: Navigation, Computer Vision"
    echo "Optional: SLAM, Autonomous Typing"
}

main() {
    local launch_mode="full"

    while [[ $# -gt 0 ]]; do
        case $1 in
            --minimal)
                launch_mode="minimal"
                shift
                ;;
            --full)
                launch_mode="full"
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

    echo "🚀 URC 2026 System Launch"
    echo "========================"
    echo "Mode: $launch_mode"
    echo ""

    trap cleanup EXIT

    check_prerequisites
    cleanup  # Clean any existing processes

    # Launch core systems (always)
    launch_state_management
    launch_safety_system

    # Launch main systems
    if [ "$launch_mode" = "full" ]; then
        launch_navigation
        launch_vision
        launch_optional_subsystems
    fi

    wait_for_system
    show_status
}

main "$@"
