#!/bin/bash
# Submodule Management Script for URC 2026 Integration
# Handles teleoperation and control systems submodules

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname "$SCRIPT_DIR")"
SUBMODULES_DIR="$WORKSPACE_ROOT/submodules"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running from correct directory
check_workspace() {
    if [[ ! -f ".gitmodules" ]]; then
        log_error "Must be run from workspace root (containing .gitmodules)"
        exit 1
    fi
}

# Update all submodules
update_submodules() {
    log_info "Updating all submodules to latest upstream..."
    git submodule update --init --recursive --remote
    log_success "Submodules updated"
}

# Update specific submodule
update_submodule() {
    local submodule="$1"
    if [[ ! -d "submodules/$submodule" ]]; then
        log_error "Submodule '$submodule' not found"
        exit 1
    fi

    log_info "Updating submodule: $submodule"
    git submodule update --remote "submodules/$submodule"
    log_success "Submodule '$submodule' updated"
}

# Check submodule status
check_status() {
    log_info "Checking submodule status..."
    git submodule status

    echo
    log_info "Submodule information:"
    echo "Teleoperation: submodules/teleoperation"
    echo "Control Systems: submodules/control-systems"

    echo
    log_info "Submodule README: submodules/README.md"
}

# Start teleoperation frontend
start_teleoperation() {
    local teleop_path="$SUBMODULES_DIR/teleoperation"

    if [[ ! -d "$teleop_path" ]]; then
        log_error "Teleoperation submodule not found. Run './scripts/manage_submodules.sh update' first"
        exit 1
    fi

    log_info "Starting teleoperation frontend..."
    cd "$teleop_path"

    # Check if node_modules exists
    if [[ ! -d "node_modules" ]]; then
        log_info "Installing dependencies..."
        npm install
    fi

    log_info "Starting development server..."
    log_info "Frontend will be available at: http://localhost:5173"
    npm run dev
}

# Show integration instructions
show_integration() {
    echo "=== URC 2026 Submodule Integration ==="
    echo
    echo "Submodules are configured as READONLY - do not make changes here!"
    echo "Make changes in the upstream repositories instead."
    echo
    echo "Available submodules:"
    echo "  - teleoperation: Web-based rover control interface"
    echo "  - control-systems: STM32-based drive and arm control"
    echo
    echo "Quick start:"
    echo "  1. Update submodules: ./scripts/manage_submodules.sh update"
    echo "  2. Start autonomy: ros2 launch mission_system system.launch.py"
    echo "  3. Start teleop: ./scripts/manage_submodules.sh teleop"
    echo "  4. Full system: ros2 launch integrated_system integrated_system.launch.py"
    echo
    echo "Testing without hardware:"
    echo "  - Use simulation: ros2 launch rover_simulation rover_gazebo.launch.py"
    echo "  - Mock controllers: control_systems_bridge will use mocks"
    echo
    echo "For hardware deployment:"
    echo "  - Control systems: Flash STM32 with stm32loader"
    echo "  - CAN bus: Ensure can0 interface is configured"
    echo "  - ROS bridge: WebSocket on port 9090"
    echo
}

# Main command handling
main() {
    check_workspace

    case "${1:-help}" in
        "update")
            if [[ -n "$2" ]]; then
                update_submodule "$2"
            else
                update_submodules
            fi
            ;;
        "status")
            check_status
            ;;
        "teleop")
            start_teleoperation
            ;;
        "integration")
            show_integration
            ;;
        "help"|*)
            echo "Usage: $0 <command> [submodule]"
            echo
            echo "Commands:"
            echo "  update [submodule]    Update all submodules or specific one"
            echo "  status                 Show submodule status"
            echo "  teleop                 Start teleoperation frontend"
            echo "  integration            Show integration instructions"
            echo "  help                   Show this help"
            echo
            show_integration
            ;;
    esac
}

main "$@"


