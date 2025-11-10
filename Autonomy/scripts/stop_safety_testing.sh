#!/bin/bash

# Safety Testing Shutdown Script

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

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

# Function to cleanup processes
cleanup_processes() {
    print_info "Stopping safety testing processes..."

    local stopped=0

    # Kill processes in reverse order of startup
    local pid_files=("/tmp/safety_frontend.pid" "/tmp/safety_system.pid" "/tmp/safety_state.pid" "/tmp/safety_rosbridge.pid")

    for pid_file in "${pid_files[@]}"; do
        if [ -f "$pid_file" ]; then
            local pid=$(cat "$pid_file")
            local process_name=$(basename "$pid_file" .pid | sed 's/safety_//')

            if kill -0 "$pid" 2>/dev/null; then
                print_info "Stopping $process_name (PID: $pid)"
                kill "$pid" 2>/dev/null || true
                sleep 1

                # Force kill if still running
                if kill -0 "$pid" 2>/dev/null; then
                    print_warning "Force stopping $process_name"
                    kill -9 "$pid" 2>/dev/null || true
                fi

                ((stopped++))
            else
                print_warning "$process_name (PID: $pid) not running"
            fi

            rm -f "$pid_file"
        fi
    done

    return $stopped
}

# Function to kill by pattern
kill_by_pattern() {
    local pattern=$1
    local description=$2

    print_info "Searching for $description processes..."

    local pids=$(pgrep -f "$pattern" || true)

    if [ -n "$pids" ]; then
        echo "$pids" | while read -r pid; do
            if kill -0 "$pid" 2>/dev/null; then
                print_info "Stopping $description (PID: $pid)"
                kill "$pid" 2>/dev/null || true
                sleep 0.5
            fi
        done
        return 0
    else
        print_info "No $description processes found"
        return 1
    fi
}

# Function to cleanup ROS2
cleanup_ros2() {
    print_info "Cleaning up ROS2 processes..."

    # Kill ROS2 nodes related to safety
    kill_by_pattern "autonomy_safety_system" "safety system nodes" || true
    kill_by_pattern "state_management" "state management nodes" || true
    kill_by_pattern "rosbridge" "ROS2 bridge" || true

    # Kill any remaining ROS2 processes that might be safety-related
    kill_by_pattern "safety_" "safety-related processes" || true
}

# Function to cleanup frontend
cleanup_frontend() {
    print_info "Cleaning up frontend processes..."

    kill_by_pattern "vite" "Vite dev server" || true
    kill_by_pattern "node.*react" "React development server" || true
}

# Function to cleanup temp files
cleanup_temp() {
    print_info "Cleaning up temporary files..."

    local temp_files=(
        "/tmp/safety_*.pid"
        "/tmp/autonomy_*.pid"
    )

    for pattern in "${temp_files[@]}"; do
        rm -f $pattern 2>/dev/null || true
    done

    print_success "Temporary files cleaned"
}

# Function to show status before cleanup
show_pre_cleanup_status() {
    echo ""
    print_info "Current process status before cleanup:"

    local processes=(
        "ROS2 bridge:rosbridge"
        "Safety system:autonomy_safety_system"
        "State manager:state_management"
        "Frontend:vite"
    )

    for proc_info in "${processes[@]}"; do
        local name=$(echo "$proc_info" | cut -d: -f1)
        local pattern=$(echo "$proc_info" | cut -d: -f2)

        local count=$(pgrep -f "$pattern" | wc -l 2>/dev/null || echo "0")
        if [ "$count" -gt 0 ]; then
            echo -e "  ${YELLOW}⚠${NC}  $name: $count process(es) running"
        else
            echo -e "  ${GREEN}✓${NC}  $name: not running"
        fi
    done
}

# Function to show final status
show_final_status() {
    echo ""
    print_info "Cleanup completed. Final status:"

    sleep 2  # Wait a bit for processes to fully terminate

    local remaining=0

    # Check for remaining safety processes
    local safety_processes=$(pgrep -f "safety_\|autonomy_safety\|rosbridge\|vite" | wc -l 2>/dev/null || echo "0")

    if [ "$safety_processes" -gt 0 ]; then
        print_warning "$safety_processes safety-related processes still running"
        echo "  Run: pkill -9 -f 'safety_\|rosbridge\|vite'  # Force kill remaining processes"
        ((remaining++))
    else
        print_success "All safety testing processes stopped"
    fi

    # Check for ROS2 processes
    local ros_processes=$(pgrep -f "ros2" | wc -l 2>/dev/null || echo "0")
    if [ "$ros_processes" -gt 0 ]; then
        print_info "$ros_processes ROS2 processes still running (normal if other ROS systems active)"
    fi

    return $remaining
}

# Function to show usage
usage() {
    echo "Usage: $0 [options]"
    echo ""
    echo "Safety Testing Environment Shutdown"
    echo ""
    echo "Options:"
    echo "  --force     Force kill all processes (use with caution)"
    echo "  --quick     Quick cleanup without detailed status"
    echo "  --help      Show this help"
    echo ""
    echo "This script stops all safety testing processes including:"
    echo "  • Safety system ROS2 nodes"
    echo "  • State management system"
    echo "  • ROS2 WebSocket bridge"
    echo "  • Safety testing frontend"
    echo "  • Temporary files and PIDs"
}

# Main execution
main() {
    local force=false
    local quick=false

    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
            --force)
                force=true
                shift
                ;;
            --quick)
                quick=false  # Override quick mode if force is used
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

    echo "🛑 URC 2026 Safety Testing Shutdown"
    echo "==================================="

    if [ "$quick" = false ]; then
        show_pre_cleanup_status
    fi

    # Perform cleanup
    local stopped_processes=0
    local force_used=false

    # Try graceful shutdown first
    cleanup_processes
    stopped_processes=$?

    cleanup_ros2
    cleanup_frontend

    # Force cleanup if requested or if graceful shutdown didn't work
    if [ "$force" = true ]; then
        print_warning "Force killing remaining processes..."
        pkill -9 -f "autonomy_safety_system" || true
        pkill -9 -f "state_management" || true
        pkill -9 -f "rosbridge" || true
        pkill -9 -f "vite" || true
        pkill -9 -f "node.*react" || true
        force_used=true
    fi

    # Cleanup temp files
    cleanup_temp

    # Show final status
    if [ "$quick" = false ]; then
        local remaining=0
        show_final_status
        remaining=$?

        echo ""
        if [ $stopped_processes -gt 0 ] || [ "$force_used" = true ]; then
            print_success "Safety testing environment shutdown completed"
            if [ "$force_used" = true ]; then
                print_warning "Force shutdown was used - some processes may have been killed abruptly"
            fi
        else
            print_info "No safety testing processes were found running"
        fi

        if [ $remaining -gt 0 ]; then
            echo ""
            print_warning "Some processes may still be running. Check with:"
            echo "  ps aux | grep -E 'safety|rosbridge|vite'"
            echo "  pgrep -f 'autonomy_safety'"
        fi
    fi
}

# Run main function
main "$@"
