#!/bin/bash

# Quick System Validation
# 2-minute basic system health check

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

ISSUES=0
WARNINGS=0

print_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
print_success() { echo -e "${GREEN}[✓]${NC} $1"; ((PASSED++)) }
print_warning() { echo -e "${YELLOW}[⚠]${NC} $1"; ((WARNINGS++)) }
print_error() { echo -e "${RED}[✗]${NC} $1"; ((ISSUES++)) }

echo "⚡ Quick System Validation (2 min)"
echo "==================================="

# Check 1: ROS2 Environment
print_info "Checking ROS2..."
if command -v ros2 &>/dev/null && ros2 topic list &>/dev/null; then
    local nodes=$(ros2 node list | wc -l)
    local topics=$(ros2 topic list | wc -l)
    print_success "ROS2 running: $nodes nodes, $topics topics"
else
    print_error "ROS2 not available or not running"
fi

# Check 2: Core Systems
print_info "Checking core systems..."
core_systems=("state_management" "safety")
for system in "${core_systems[@]}"; do
    if ros2 node list 2>/dev/null | grep -q "$system"; then
        print_success "$system running"
    else
        print_error "$system not found"
    fi
done

# Check 3: Critical Topics
print_info "Checking critical topics..."
critical_topics=("/state_machine/current_state" "/safety/dashboard_status" "/clock")
for topic in "${critical_topics[@]}"; do
    if ros2 topic info "$topic" &>/dev/null; then
        print_success "$topic available"
    else
        print_error "$topic missing"
    fi
done

# Check 4: Safety System
print_info "Checking safety system..."
if ros2 topic echo /safety/dashboard_status --once --timeout 3 &>/dev/null; then
    print_success "Safety system responding"
else
    print_warning "Safety system not responding"
fi

# Check 5: System Resources
print_info "Checking system resources..."
local mem_usage=$(free | grep Mem | awk '{printf "%.0f", $3/$2 * 100.0}')
if [ "$mem_usage" -lt 90 ]; then
    print_success "Memory usage: ${mem_usage}%"
else
    print_error "High memory usage: ${mem_usage}%"
fi

# Summary
echo ""
echo "📊 Quick Validation Results:"
echo "  Passed: $((${PASSED:-0}))"
echo "  Warnings: $WARNINGS"
echo "  Issues: $ISSUES"

if [ $ISSUES -eq 0 ]; then
    if [ $WARNINGS -eq 0 ]; then
        print_success "🎉 System ready!"
    else
        print_warning "⚠️ System OK with warnings"
    fi
else
    print_error "❌ Issues found - run full validation"
    echo ""
    echo "Next steps:"
    echo "  ./scripts/health_check.sh     # Detailed health check"
    echo "  ./scripts/validate_deploy.sh  # Deployment validation"
fi

echo ""
echo "⏱️ Validation completed in ~30 seconds"
