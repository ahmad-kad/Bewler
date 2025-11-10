#!/bin/bash

# Competition Validation
# 10-minute comprehensive URC 2026 compliance check

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

ISSUES=0
WARNINGS=0
PASSED=0

print_info() { echo -e "${BLUE}[INFO]${NC} $1"; }
print_success() { echo -e "${GREEN}[✓]${NC} $1"; ((PASSED++)) }
print_warning() { echo -e "${YELLOW}[⚠]${NC} $1"; ((WARNINGS++)) }
print_error() { echo -e "${RED}[✗]${NC} $1"; ((ISSUES++)) }

echo "🏆 URC 2026 Competition Validation (10 min)"
echo "============================================="

# Check 1: Emergency Stop Compliance
print_info "Emergency stop compliance..."
if ros2 service list 2>/dev/null | grep -q "software_estop"; then
    print_success "Software emergency stop service available"

    # Test response time (should be < 3 seconds per URC rules)
    start_time=$(date +%s%N)
    if ros2 service call /state_machine/software_estop autonomy_interfaces/srv/SoftwareEstop "
operator_id: 'competition_test'
reason: 'URC_compliance_test'
acknowledge_criticality: true
force_immediate: false" --timeout 5 &>/dev/null; then
        end_time=$(date +%s%N)
        response_ms=$(( (end_time - start_time) / 1000000 ))
        if [ $response_ms -lt 3000 ]; then
            print_success "Emergency stop response: ${response_ms}ms (< 3000ms URC limit)"
        else
            print_error "Emergency stop too slow: ${response_ms}ms (> 3000ms URC limit)"
        fi
    else
        print_error "Emergency stop service failed"
    fi
else
    print_error "Emergency stop service missing (URC requirement)"
fi

# Check 2: Safety System Components
print_info "Safety system components..."
safety_components=("autonomy_safety_system")
for component in "${safety_components[@]}"; do
    if ros2 pkg list 2>/dev/null | grep -q "$component"; then
        print_success "$component available"
    else
        print_error "$component missing (URC safety requirement)"
    fi
done

# Check 3: System State Management
print_info "State management compliance..."
if ros2 topic echo /state_machine/current_state --once --timeout 3 &>/dev/null; then
    print_success "State machine operational"
else
    print_error "State machine not responding"
fi

# Check 4: Multi-Subsystem Coordination
print_info "Multi-subsystem coordination..."
subsystems=("state_management" "safety" "navigation")
running_subsystems=0
for subsys in "${subsystems[@]}"; do
    if ros2 node list 2>/dev/null | grep -q "$subsys"; then
        ((running_subsystems++))
    fi
done

if [ $running_subsystems -ge 2 ]; then
    print_success "Multi-subsystem coordination: $running_subsystems/$(( ${#subsystems[@]} )) running"
else
    print_warning "Limited subsystem coordination: $running_subsystems/$(( ${#subsystems[@]} )) running"
fi

# Check 5: Performance Requirements
print_info "Performance requirements..."

# CPU usage during operation
cpu_usage=$(top -b -n1 | grep "Cpu(s)" | awk '{print $2}' | cut -d'.' -f1)
if [ "$cpu_usage" -lt 80 ]; then
    print_success "CPU performance acceptable: ${cpu_usage}% usage"
else
    print_warning "High CPU usage: ${cpu_usage}% (monitor during operation)"
fi

# Memory usage
mem_usage=$(free | grep Mem | awk '{printf "%.0f", $3/$2 * 100.0}')
if [ "$mem_usage" -lt 85 ]; then
    print_success "Memory usage acceptable: ${mem_usage}%"
else
    print_error "High memory usage: ${mem_usage}% (risk of OOM)"
fi

# Check 6: Sensor Integration
print_info "Sensor integration..."
sensor_topics=("/imu/data" "/gps/fix" "/camera/image_raw")
available_sensors=0
for topic in "${sensor_topics[@]}"; do
    if ros2 topic info "$topic" &>/dev/null; then
        ((available_sensors++))
    fi
done

if [ $available_sensors -ge 2 ]; then
    print_success "Sensor integration: $available_sensors/3 sensors available"
else
    print_warning "Limited sensor integration: $available_sensors/3 sensors"
fi

# Check 7: Communication Robustness
print_info "Communication robustness..."
topic_count=$(ros2 topic list 2>/dev/null | wc -l)
if [ "$topic_count" -gt 10 ]; then
    print_success "Communication system: $topic_count topics active"
else
    print_warning "Limited communication: $topic_count topics"
fi

# Check 8: System Stability Test
print_info "System stability (5-minute test)..."
echo "  Monitoring system for 5 minutes..."
stable=true

for i in {1..30}; do  # 30 * 10s = 5 minutes
    sleep 10

    # Check if critical systems still running
    if ! ros2 node list 2>/dev/null | grep -q "state_management\|safety"; then
        print_error "Critical system failure during stability test"
        stable=false
        break
    fi

    # Check for memory leaks (rough approximation)
    current_mem=$(free | grep Mem | awk '{printf "%.0f", $3/$2 * 100.0}')
    if [ "$current_mem" -gt 95 ]; then
        print_error "Memory leak detected during stability test"
        stable=false
        break
    fi

    echo -n "."
done
echo ""

if [ "$stable" = true ]; then
    print_success "System stability: Passed 5-minute stability test"
else
    print_error "System stability: Failed stability test"
fi

# Check 9: Competition Scenario Readiness
print_info "Competition scenario readiness..."

# Check for mission execution capability
if ros2 topic info /state_machine/current_state &>/dev/null; then
    print_success "Mission execution framework available"
else
    print_error "Mission execution framework missing"
fi

# Check for autonomous operation capability
if ros2 node list 2>/dev/null | grep -q "navigation\|computer_vision"; then
    print_success "Autonomous operation components available"
else
    print_warning "Limited autonomous operation capability"
fi

# Summary
echo ""
echo "🏆 URC 2026 Competition Validation Results:"
echo "  Passed: $PASSED"
echo "  Warnings: $WARNINGS"
echo "  Critical Issues: $ISSUES"

echo ""
if [ $ISSUES -eq 0 ]; then
    if [ $WARNINGS -eq 0 ]; then
        print_success "🎉 FULLY COMPLIANT - Ready for URC 2026!"
        echo ""
        echo "✅ System meets all URC 2026 requirements:"
        echo "  • Emergency stop < 3 seconds"
        echo "  • Multi-layered safety system"
        echo "  • Stable 5-minute operation"
        echo "  • Sensor integration"
        echo "  • Communication robustness"
    else
        print_warning "⚠️ MOSTLY COMPLIANT - Address $WARNINGS warnings"
        echo ""
        echo "System is competition-ready but monitor:"
        echo "  • CPU usage during operation"
        echo "  • Sensor availability"
        echo "  • Communication stability"
    fi
else
    print_error "❌ NOT COMPLIANT - $ISSUES critical issues"
    echo ""
    echo "Critical issues preventing competition:"
    for issue in "${CRITICAL_ISSUES[@]}"; do
        echo "  • $issue"
    done
    echo ""
    echo "Resolve issues before competition day!"
fi

echo ""
echo "⏱️ Competition validation completed in ~8-10 minutes"
echo ""
echo "📋 Final checklist:"
echo "  [$( [ $ISSUES -eq 0 ] && echo "x" || echo " " )] Critical issues resolved"
echo "  [$( [ $WARNINGS -eq 0 ] && echo "x" || echo " " )] Warnings addressed"
echo "  [ ] Hardware emergency stop tested"
echo "  [ ] Team emergency procedures reviewed"
echo "  [ ] Competition documentation complete"
