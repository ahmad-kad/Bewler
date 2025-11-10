#!/bin/bash

# System Stop Script
# Clean shutdown of autonomy system

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
print_success() { echo -e "${GREEN}[✓]${NC} $1"; }
print_warning() { echo -e "${YELLOW}[⚠]${NC} $1"; }

echo "🛑 Autonomy System Shutdown"
echo "==========================="

# Stop processes in reverse order
pids=(
    "/tmp/system_typing.pid"
    "/tmp/system_slam.pid"
    "/tmp/system_vision.pid"
    "/tmp/system_nav.pid"
    "/tmp/system_safety.pid"
    "/tmp/system_state.pid"
)

stopped=0
for pid_file in "${pids[@]}"; do
    if [ -f "$pid_file" ]; then
        pid=$(cat "$pid_file")
        process_name=$(basename "$pid_file" .pid | sed 's/system_//')

        if kill -0 "$pid" 2>/dev/null; then
            print_info "Stopping $process_name (PID: $pid)"
            kill "$pid" 2>/dev/null || true
            sleep 1

            # Force kill if still running
            if kill -0 "$pid" 2>/dev/null; then
                kill -9 "$pid" 2>/dev/null || true
            fi
            ((stopped++))
        fi

        rm -f "$pid_file"
    fi
done

# Kill any remaining autonomy processes
print_info "Cleaning up remaining autonomy processes..."
pkill -f "autonomy_" || true
pkill -f "ros2.*launch.*autonomy" || true
sleep 2

# Verify shutdown
remaining=$(pgrep -f "autonomy_" | wc -l)
if [ "$remaining" -eq 0 ]; then
    print_success "All autonomy processes stopped"
else
    print_warning "$remaining autonomy processes still running"
fi

echo ""
if [ $stopped -gt 0 ]; then
    print_success "System shutdown completed - stopped $stopped processes"
else
    print_info "No running autonomy processes found"
fi

echo ""
echo "System ready for restart:"
echo "  ./scripts/system_launch.sh    # Restart system"
echo "  ./scripts/health_check.sh     # Verify shutdown"
