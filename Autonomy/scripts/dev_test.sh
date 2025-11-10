#!/bin/bash

# Development Testing Helper Script
# Quick testing during development iterations

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

# Function to check development environment
check_dev_env() {
    print_header "Development Environment Check"

    # Check if in correct directory
    if [ ! -d "$ROS_WS" ]; then
        print_error "Not in correct project directory"
        echo "  Run from: $PROJECT_ROOT"
        exit 1
    fi

    # Check ROS2
    if ! command -v ros2 &> /dev/null; then
        print_error "ROS2 not sourced"
        echo "  Run: source /opt/ros/humble/setup.bash"
        exit 1
    fi
    print_success "ROS2 available"

    # Check workspace
    if [ ! -f "$ROS_WS/install/setup.bash" ]; then
        print_warning "Workspace not built"
        echo "  Run: cd ros2_ws && colcon build"
    fi
}

# Function to build specific package
build_package() {
    local package=$1

    if [ -z "$package" ]; then
        print_error "No package specified"
        return 1
    fi

    print_info "Building package: $package"

    cd "$ROS_WS"
    if colcon build --packages-select "$package" --continue-on-error; then
        print_success "Package $package built successfully"
        source install/setup.bash
        return 0
    else
        print_error "Failed to build package $package"
        return 1
    fi
}

# Function to run unit tests for package
run_unit_tests() {
    local package=$1

    if [ -z "$package" ]; then
        print_error "No package specified"
        return 1
    fi

    print_info "Running unit tests for: $package"

    # Source workspace
    cd "$ROS_WS"
    source install/setup.bash

    # Run tests
    if python -m pytest "../code/$package/test/" -v --tb=short; then
        print_success "Unit tests passed for $package"
        return 0
    else
        print_error "Unit tests failed for $package"
        return 1
    fi
}

# Function to run integration tests
run_integration_tests() {
    local subsystem=$1

    print_info "Running integration tests"

    # Source workspace
    cd "$ROS_WS"
    source install/setup.bash

    # Run relevant integration tests
    if [ -n "$subsystem" ]; then
        if python -m pytest "../tests/integration/" -k "$subsystem" -v --tb=short; then
            print_success "Integration tests passed for $subsystem"
            return 0
        else
            print_error "Integration tests failed for $subsystem"
            return 1
        fi
    else
        if python -m pytest "../tests/integration/" -v --tb=short; then
            print_success "All integration tests passed"
            return 0
        else
            print_error "Integration tests failed"
            return 1
        fi
    fi
}

# Function to launch subsystem for testing
launch_subsystem() {
    local subsystem=$1

    if [ -z "$subsystem" ]; then
        print_error "No subsystem specified"
        return 1
    fi

    print_info "Launching subsystem: $subsystem"

    # Source workspace
    cd "$ROS_WS"
    source install/setup.bash

    # Launch appropriate subsystem
    case $subsystem in
        "navigation")
            ros2 launch autonomy_navigation navigation.launch.py &
            ;;
        "vision")
            ros2 launch autonomy_computer_vision computer_vision.launch.py &
            ;;
        "slam")
            ros2 launch autonomy_slam slam.launch.py &
            ;;
        "state")
            ros2 launch autonomy_state_management state_machine.launch.py &
            ;;
        "safety")
            ros2 launch autonomy_safety_system safety_system.launch.py &
            ;;
        "typing")
            ros2 launch autonomy_autonomous_typing typing_system.launch.py &
            ;;
        *)
            print_error "Unknown subsystem: $subsystem"
            echo "  Available: navigation, vision, slam, state, safety, typing"
            return 1
            ;;
    esac

    local pid=$!
    echo $pid > "/tmp/dev_test_${subsystem}.pid"
    print_success "Subsystem $subsystem launched (PID: $pid)"

    # Wait a bit for startup
    sleep 3

    # Quick health check
    local node_count=$(ros2 node list 2>/dev/null | grep -c "$subsystem" || echo "0")
    if [ "$node_count" -gt "0" ]; then
        print_success "$subsystem subsystem healthy ($node_count nodes)"
    else
        print_warning "$subsystem subsystem may not be fully started"
    fi
}

# Function to test subsystem functionality
test_subsystem() {
    local subsystem=$1

    if [ -z "$subsystem" ]; then
        print_error "No subsystem specified"
        return 1
    fi

    print_info "Testing subsystem functionality: $subsystem"

    case $subsystem in
        "navigation")
            # Test navigation topics
            if ros2 topic info /navigation/cmd_vel &>/dev/null; then
                print_success "Navigation topics available"
                # Send test velocity command
                ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "
linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
" && print_success "Navigation command accepted"
            else
                print_error "Navigation topics not available"
                return 1
            fi
            ;;
        "vision")
            # Test vision topics
            if ros2 topic hz /camera/image_raw --duration 2 &>/dev/null; then
                print_success "Camera feed active"
            else
                print_warning "Camera feed not active"
            fi
            ;;
        "safety")
            # Test safety topics
            if ros2 topic echo /safety/dashboard_status --once --timeout 3 &>/dev/null; then
                print_success "Safety system responding"
            else
                print_error "Safety system not responding"
                return 1
            fi
            ;;
        "state")
            # Test state machine
            if ros2 topic echo /state_machine/current_state --once --timeout 3 &>/dev/null; then
                print_success "State machine responding"
            else
                print_error "State machine not responding"
                return 1
            fi
            ;;
        *)
            print_info "Basic connectivity test for $subsystem"
            # Generic test - check if any related topics exist
            local topic_count=$(ros2 topic list 2>/dev/null | grep -c "$subsystem" || echo "0")
            if [ "$topic_count" -gt "0" ]; then
                print_success "$subsystem has $topic_count active topics"
            else
                print_warning "No active topics found for $subsystem"
            fi
            ;;
    esac
}

# Function to smoke test (quick functionality check)
smoke_test() {
    local subsystem=$1

    print_info "Running smoke test for $subsystem"

    # Launch subsystem
    if ! launch_subsystem "$subsystem"; then
        print_error "Failed to launch $subsystem"
        return 1
    fi

    # Test functionality
    if test_subsystem "$subsystem"; then
        print_success "Smoke test passed for $subsystem"
        return 0
    else
        print_error "Smoke test failed for $subsystem"
        return 1
    fi
}

# Function to cleanup development testing
cleanup_dev_test() {
    print_info "Cleaning up development test processes..."

    # Kill any launched subsystems
    local pid_files=$(ls /tmp/dev_test_*.pid 2>/dev/null || true)

    for pid_file in $pid_files; do
        if [ -f "$pid_file" ]; then
            local pid=$(cat "$pid_file")
            local subsystem=$(basename "$pid_file" .pid | sed 's/dev_test_//')

            if kill -0 "$pid" 2>/dev/null; then
                print_info "Stopping $subsystem (PID: $pid)"
                kill "$pid" 2>/dev/null || true
                sleep 1
            fi

            rm -f "$pid_file"
        fi
    done

    # Kill any remaining test processes
    pkill -f "ros2.*launch.*autonomy" || true

    print_success "Cleanup completed"
}

# Function to show development workflow
show_workflow() {
    print_header "Development Testing Workflow"

    echo "Typical development iteration:"
    echo ""
    echo "1. Make code changes in code/ directory"
    echo "2. Build specific package:"
    echo "   ./scripts/dev_test.sh build <package>"
    echo ""
    echo "3. Run unit tests:"
    echo "   ./scripts/dev_test.sh unit <package>"
    echo ""
    echo "4. Launch subsystem for testing:"
    echo "   ./scripts/dev_test.sh launch <subsystem>"
    echo ""
    echo "5. Test functionality:"
    echo "   ./scripts/dev_test.sh test <subsystem>"
    echo ""
    echo "6. Run smoke test (launch + test):"
    echo "   ./scripts/dev_test.sh smoke <subsystem>"
    echo ""
    echo "Available subsystems:"
    echo "  navigation, vision, slam, state, safety, typing"
    echo ""
    echo "Quick commands:"
    echo "  ./scripts/dev_test.sh build autonomy_navigation"
    echo "  ./scripts/dev_test.sh smoke navigation"
    echo "  ./scripts/dev_test.sh cleanup"
}

# Function to show usage
usage() {
    echo "Usage: $0 <command> [subsystem|package]"
    echo ""
    echo "Development Testing Helper for URC 2026 Autonomy"
    echo ""
    echo "Commands:"
    echo "  build <package>     Build specific ROS2 package"
    echo "  unit <package>      Run unit tests for package"
    echo "  integration [sub]   Run integration tests (optionally for subsystem)"
    echo "  launch <subsystem>  Launch subsystem for testing"
    echo "  test <subsystem>    Test subsystem functionality"
    echo "  smoke <subsystem>   Launch + test subsystem (smoke test)"
    echo "  cleanup             Clean up test processes"
    echo "  workflow            Show development workflow"
    echo "  help                Show this help"
    echo ""
    echo "Subsystems: navigation, vision, slam, state, safety, typing"
    echo "Packages: autonomy_navigation, autonomy_computer_vision, etc."
    echo ""
    echo "Examples:"
    echo "  $0 build autonomy_navigation"
    echo "  $0 smoke navigation"
    echo "  $0 integration safety"
    echo "  $0 cleanup"
}

# Main execution
main() {
    local command=$1
    local target=$2

    echo "🔧 URC 2026 Development Testing Helper"
    echo "======================================"

    check_dev_env

    case $command in
        "build")
            if [ -z "$target" ]; then
                print_error "Package name required"
                echo "  Usage: $0 build <package_name>"
                exit 1
            fi
            build_package "$target"
            ;;
        "unit")
            if [ -z "$target" ]; then
                print_error "Package name required"
                echo "  Usage: $0 unit <package_name>"
                exit 1
            fi
            run_unit_tests "$target"
            ;;
        "integration")
            run_integration_tests "$target"
            ;;
        "launch")
            if [ -z "$target" ]; then
                print_error "Subsystem name required"
                echo "  Usage: $0 launch <subsystem>"
                echo "  Available: navigation, vision, slam, state, safety, typing"
                exit 1
            fi
            launch_subsystem "$target"
            ;;
        "test")
            if [ -z "$target" ]; then
                print_error "Subsystem name required"
                echo "  Usage: $0 test <subsystem>"
                echo "  Available: navigation, vision, slam, state, safety, typing"
                exit 1
            fi
            test_subsystem "$target"
            ;;
        "smoke")
            if [ -z "$target" ]; then
                print_error "Subsystem name required"
                echo "  Usage: $0 smoke <subsystem>"
                echo "  Available: navigation, vision, slam, state, safety, typing"
                exit 1
            fi
            smoke_test "$target"
            ;;
        "cleanup")
            cleanup_dev_test
            ;;
        "workflow")
            show_workflow
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

# Trap for cleanup on exit
trap cleanup_dev_test EXIT

# Run main function
main "$@"
