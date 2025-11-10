#!/bin/bash

# System Health Check Script
# Quick validation of autonomy system health

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
ISSUES=0
WARNINGS=0

# Function to print colored output
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[✓]${NC} $1"
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
    echo -e "${BLUE}$(printf '%.0s=' {1..50})${NC}"
}

# Function to check ROS2 environment
check_ros2() {
    print_header "ROS2 Environment Check"

    # Check if ROS2 is sourced
    if ! command -v ros2 &> /dev/null; then
        print_error "ROS2 command not found"
        echo "  Run: source /opt/ros/humble/setup.bash"
        return 1
    fi
    print_success "ROS2 available"

    # Check ROS2 daemon
    if ! ros2 daemon status &> /dev/null; then
        print_warning "ROS2 daemon not running"
        echo "  Starting daemon..."
        ros2 daemon start
        sleep 2
    else
        print_success "ROS2 daemon running"
    fi

    # Check workspace
    if [ ! -d "$PROJECT_ROOT/ros2_ws/install" ]; then
        print_warning "ROS2 workspace not built"
        echo "  Run: cd ros2_ws && colcon build"
    else
        print_success "ROS2 workspace built"
    fi
}

# Function to check system nodes
check_nodes() {
    print_header "System Nodes Check"

    local nodes_running=$(ros2 node list 2>/dev/null | wc -l)
    echo "Total nodes running: $nodes_running"

    # Critical nodes that should be running
    local critical_nodes=("state_management" "safety" "navigation" "computer_vision")

    for node in "${critical_nodes[@]}"; do
        if ros2 node list 2>/dev/null | grep -q "$node"; then
            print_success "$node node running"
        else
            print_error "$node node not found"
        fi
    done

    # Optional nodes
    local optional_nodes=("slam" "autonomous_typing" "sensor_bridge")

    for node in "${optional_nodes[@]}"; do
        if ros2 node list 2>/dev/null | grep -q "$node"; then
            print_success "$node node running (optional)"
        fi
    done
}

# Function to check topics
check_topics() {
    print_header "Topic Availability Check"

    local total_topics=$(ros2 topic list 2>/dev/null | wc -l)
    echo "Total topics: $total_topics"

    # Critical topics
    local critical_topics=(
        "/state_machine/current_state"
        "/safety/dashboard_status"
        "/navigation/cmd_vel"
        "/clock"
    )

    for topic in "${critical_topics[@]}"; do
        if ros2 topic info "$topic" &>/dev/null; then
            print_success "$topic available"
        else
            print_error "$topic missing"
        fi
    done

    # Check topic frequencies for active topics
    print_info "Checking topic frequencies..."
    local active_topics=("/state_machine/current_state" "/safety/dashboard_status")

    for topic in "${active_topics[@]}"; do
        if ros2 topic info "$topic" &>/dev/null; then
            local hz=$(timeout 5 ros2 topic hz "$topic" 2>/dev/null | tail -1 | awk '{print $2}' || echo "0")
            if (( $(echo "$hz > 0" | bc -l 2>/dev/null || echo "0") )); then
                print_success "$topic: ${hz}Hz"
            else
                print_warning "$topic: no data (may be normal)"
            fi
        fi
    done
}

# Function to check services
check_services() {
    print_header "Service Availability Check"

    local total_services=$(ros2 service list 2>/dev/null | wc -l)
    echo "Total services: $total_services"

    # Critical services
    local critical_services=(
        "/state_machine/change_state"
        "/state_machine/get_system_state"
    )

    for service in "${critical_services[@]}"; do
        if ros2 service info "$service" &>/dev/null; then
            print_success "$service available"
        else
            print_error "$service missing"
        fi
    done
}

# Function to check system resources
check_resources() {
    print_header "System Resources Check"

    # CPU usage
    local cpu_idle=$(top -b -n1 | grep "Cpu(s)" | awk '{print $8}' | cut -d'.' -f1)
    local cpu_usage=$((100 - cpu_idle))

    if [ $cpu_usage -lt 80 ]; then
        print_success "CPU usage: ${cpu_usage}%"
    else
        print_warning "High CPU usage: ${cpu_usage}%"
    fi

    # Memory usage
    local mem_usage=$(free | grep Mem | awk '{printf "%.0f", $3/$2 * 100.0}')

    if [ $mem_usage -lt 90 ]; then
        print_success "Memory usage: ${mem_usage}%"
    else
        print_warning "High memory usage: ${mem_usage}%"
    fi

    # Disk space
    local disk_usage=$(df "$PROJECT_ROOT" | tail -1 | awk '{print $5}' | sed 's/%//')

    if [ $disk_usage -lt 90 ]; then
        print_success "Disk usage: ${disk_usage}%"
    else
        print_warning "High disk usage: ${disk_usage}%"
    fi
}

# Function to check safety system
check_safety() {
    print_header "Safety System Check"

    # Check safety dashboard
    if ros2 topic echo /safety/dashboard_status --once --timeout 3 &>/dev/null; then
        print_success "Safety dashboard responding"

        # Parse safety status
        local safety_data=$(ros2 topic echo /safety/dashboard_status --once --timeout 3 2>/dev/null | head -20)
        if echo "$safety_data" | grep -q "overall_status.*NORMAL"; then
            print_success "Safety status: NORMAL"
        elif echo "$safety_data" | grep -q "overall_status.*WARNING"; then
            print_warning "Safety status: WARNING"
        else
            print_error "Safety status: CRITICAL or unknown"
        fi
    else
        print_warning "Safety dashboard not responding"
    fi

    # Check emergency stop service
    if ros2 service list 2>/dev/null | grep -q "software_estop"; then
        print_success "Emergency stop service available"
    else
        print_error "Emergency stop service missing"
    fi
}

# Function to check navigation system
check_navigation() {
    print_header "Navigation System Check"

    # Check navigation topics
    local nav_topics=("/navigation/cmd_vel" "/odom")

    for topic in "${nav_topics[@]}"; do
        if ros2 topic info "$topic" &>/dev/null; then
            print_success "Navigation topic $topic available"
        else
            print_warning "Navigation topic $topic missing"
        fi
    done

    # Check if navigation is publishing commands
    if ros2 topic hz /navigation/cmd_vel --duration 2 &>/dev/null | grep -q "average rate"; then
        print_success "Navigation commands being published"
    else
        print_info "Navigation not actively publishing (may be normal)"
    fi
}

# Function to check vision system
check_vision() {
    print_header "Computer Vision System Check"

    # Check camera topics
    if ros2 topic info /camera/image_raw &>/dev/null; then
        print_success "Camera feed available"
    else
        print_warning "Camera feed not available"
    fi

    # Check vision processing topics
    local vision_topics=("/vision/detections" "/vision/aruco_markers")

    for topic in "${vision_topics[@]}"; do
        if ros2 topic info "$topic" &>/dev/null; then
            print_success "Vision topic $topic available"
        else
            print_info "Vision topic $topic not available (may be normal)"
        fi
    done
}

# Function to generate summary
generate_summary() {
    print_header "Health Check Summary"

    echo "Issues found: $ISSUES"
    echo "Warnings: $WARNINGS"

    echo ""
    if [ $ISSUES -eq 0 ]; then
        if [ $WARNINGS -eq 0 ]; then
            print_success "🎉 System is healthy!"
        else
            print_warning "⚠️ System is mostly healthy ($WARNINGS warnings)"
        fi
    else
        print_error "❌ System has issues ($ISSUES critical issues)"
        echo ""
        echo "Recommendations:"
        echo "1. Check ROS2 environment setup"
        echo "2. Ensure all required nodes are launched"
        echo "3. Verify sensor connections"
        echo "4. Run full system integration test"
        echo ""
        echo "For help: ./scripts/test_full_system.sh --help"
    fi

    echo ""
    echo "Quick commands:"
    echo "  ./scripts/test_full_system.sh --quick    # Quick validation"
    echo "  ./scripts/start_safety_testing.sh        # Safety testing"
    echo "  ./scripts/health_check.sh               # This check"
}

# Function to show usage
usage() {
    echo "Usage: $0 [options]"
    echo ""
    echo "System Health Check for URC 2026 Autonomy"
    echo ""
    echo "Options:"
    echo "  --quick        Quick check (skip detailed analysis)"
    echo "  --verbose      Verbose output"
    echo "  --help         Show this help"
    echo ""
    echo "Exit codes:"
    echo "  0 = System healthy"
    echo "  1 = Critical issues found"
    echo "  2 = Warnings only"
}

# Main execution
main() {
    local quick=false
    local verbose=false

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --quick)
                quick=true
                shift
                ;;
            --verbose)
                verbose=true
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

    echo "💚 URC 2026 System Health Check"
    echo "==============================="

    # Run checks
    check_ros2

    if [ "$quick" = false ]; then
        check_nodes
        check_topics
        check_services
        check_resources
        check_safety
        check_navigation
        check_vision
    fi

    generate_summary

    # Exit with appropriate code
    if [ $ISSUES -gt 0 ]; then
        exit 1
    elif [ $WARNINGS -gt 0 ]; then
        exit 2
    else
        exit 0
    fi
}

# Run main function
main "$@"
