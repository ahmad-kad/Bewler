#!/bin/bash

# System Performance Benchmarking Script
# Measures and analyzes autonomy system performance

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
DURATION=${1:-30}  # Benchmark duration in seconds
OUTPUT_FILE="benchmark_report_$(date +%Y%m%d_%H%M%S).json"

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
    echo -e "${BLUE}$(printf '%.0s=' {1..60})${NC}"
}

# Function to check prerequisites
check_prerequisites() {
    print_header "Checking Prerequisites"

    # Check ROS2
    if ! command -v ros2 &> /dev/null; then
        print_error "ROS2 not found"
        exit 1
    fi
    print_success "ROS2 available"

    # Check if system is running
    if ! ros2 node list &>/dev/null; then
        print_error "No ROS2 system detected. Start autonomy system first."
        echo "  Run: ./scripts/test_full_system.sh --no-cleanup"
        exit 1
    fi

    local node_count=$(ros2 node list | wc -l)
    print_success "ROS2 system running ($node_count nodes)"

    # Check benchmark tools
    if ! command -v bc &> /dev/null; then
        print_warning "bc not available - some calculations may fail"
    fi
}

# Function to collect system metrics
collect_system_metrics() {
    print_info "Collecting system performance metrics..."

    # CPU usage
    local cpu_user=$(top -b -n1 | grep "Cpu(s)" | awk '{print $2}' | cut -d'.' -f1)
    local cpu_system=$(top -b -n1 | grep "Cpu(s)" | awk '{print $4}' | cut -d'.' -f1)
    local cpu_idle=$(top -b -n1 | grep "Cpu(s)" | awk '{print $8}' | cut -d'.' -f1)

    # Memory usage
    local mem_total=$(free -m | grep Mem | awk '{print $2}')
    local mem_used=$(free -m | grep Mem | awk '{print $3}')
    local mem_free=$(free -m | grep Mem | awk '{print $4}')

    # Disk I/O (if available)
    local disk_read=$(iostat -d 1 1 2>/dev/null | grep -A1 "Device" | tail -1 | awk '{print $2}' || echo "0")
    local disk_write=$(iostat -d 1 1 2>/dev/null | grep -A1 "Device" | tail -1 | awk '{print $3}' || echo "0")

    # Network I/O (if available)
    local net_rx=$(cat /proc/net/dev | grep -E "eth0|wlan0|enp" | head -1 | awk '{print $2}' || echo "0")
    local net_tx=$(cat /proc/net/dev | grep -E "eth0|wlan0|enp" | head -1 | awk '{print $10}' || echo "0")

    cat << EOF
{
  "timestamp": "$(date +%s)",
  "system": {
    "cpu": {
      "user": $cpu_user,
      "system": $cpu_system,
      "idle": $cpu_idle,
      "total_used": $((cpu_user + cpu_system))
    },
    "memory": {
      "total_mb": $mem_total,
      "used_mb": $mem_used,
      "free_mb": $mem_free,
      "usage_percent": $(echo "scale=2; $mem_used * 100 / $mem_total" | bc -l 2>/dev/null || echo "0")
    },
    "disk": {
      "read_kb": $disk_read,
      "write_kb": $disk_write
    },
    "network": {
      "rx_bytes": $net_rx,
      "tx_bytes": $net_tx
    }
  }
EOF
}

# Function to collect ROS2 metrics
collect_ros2_metrics() {
    print_info "Collecting ROS2 performance metrics..."

    # Node count
    local node_count=$(ros2 node list 2>/dev/null | wc -l)

    # Topic count and frequencies
    local topic_count=$(ros2 topic list 2>/dev/null | wc -l)

    # Critical topic frequencies
    declare -A topic_frequencies
    local critical_topics=(
        "/state_machine/current_state"
        "/navigation/cmd_vel"
        "/odom"
        "/imu/data"
        "/camera/image_raw"
        "/safety/dashboard_status"
    )

    for topic in "${critical_topics[@]}"; do
        if ros2 topic info "$topic" &>/dev/null; then
            local hz=$(timeout 3 ros2 topic hz "$topic" 2>/dev/null | tail -1 | awk '{print $2}' || echo "0")
            topic_frequencies["$topic"]=$hz
        else
            topic_frequencies["$topic"]="null"
        fi
    done

    # Service count
    local service_count=$(ros2 service list 2>/dev/null | wc -l)

    # Message queue depths (if available)
    local queue_info="{}"

    cat << EOF
{
  "ros2": {
    "nodes": {
      "count": $node_count
    },
    "topics": {
      "count": $topic_count,
      "frequencies": {
EOF

    local first=true
    for topic in "${critical_topics[@]}"; do
        if [ "$first" = true ]; then
            first=false
        else
            echo ","
        fi
        echo "        \"$topic\": ${topic_frequencies[$topic]}"
    done

    cat << EOF
      }
    },
    "services": {
      "count": $service_count
    },
    "queues": $queue_info
  }
EOF
}

# Function to run performance tests
run_performance_tests() {
    print_info "Running performance benchmark tests..."

    # State machine transition test
    local state_test_result=$(test_state_machine_performance)

    # Navigation computation test
    local nav_test_result=$(test_navigation_performance)

    # Vision processing test
    local vision_test_result=$(test_vision_performance)

    # Safety system test
    local safety_test_result=$(test_safety_performance)

    cat << EOF
{
  "performance_tests": {
    "state_machine": $state_test_result,
    "navigation": $nav_test_result,
    "vision": $vision_test_result,
    "safety": $safety_test_result
  }
EOF
}

# Function to test state machine performance
test_state_machine_performance() {
    print_info "Testing state machine performance..."

    local start_time=$(date +%s%N)
    local transitions=0
    local errors=0

    # Test multiple state transitions
    for i in {1..5}; do
        if ros2 service call /state_machine/change_state autonomy_interfaces/srv/ChangeState "
target_state: 'IDLE'
reason: 'performance_test_$i'
" &>/dev/null; then
            ((transitions++))
        else
            ((errors++))
        fi
        sleep 0.1
    done

    local end_time=$(date +%s%N)
    local duration=$(( (end_time - start_time) / 1000000 ))  # milliseconds

    cat << EOF
{
  "transitions_attempted": 5,
  "transitions_successful": $transitions,
  "errors": $errors,
  "total_duration_ms": $duration,
  "average_latency_ms": $(( duration / 5 ))
}
EOF
}

# Function to test navigation performance
test_navigation_performance() {
    print_info "Testing navigation performance..."

    # Test path planning speed (if navigation is running)
    if ros2 node list 2>/dev/null | grep -q navigation; then
        local start_time=$(date +%s%N)

        # Send navigation goal
        ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
pose:
  header:
    frame_id: 'map'
  pose:
    position:
      x: 1.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
" 2>/dev/null &
        local goal_pid=$!

        # Wait a bit for processing
        sleep 2
        kill $goal_pid 2>/dev/null || true

        local end_time=$(date +%s%N)
        local duration=$(( (end_time - start_time) / 1000000 ))  # milliseconds

        cat << EOF
{
  "goal_processing_time_ms": $duration,
  "navigation_available": true
}
EOF
    else
        cat << EOF
{
  "navigation_available": false,
  "error": "Navigation system not running"
}
EOF
    fi
}

# Function to test vision performance
test_vision_performance() {
    print_info "Testing vision system performance..."

    if ros2 topic hz /camera/image_raw --duration 2 2>/dev/null | grep -q "average rate"; then
        local fps=$(ros2 topic hz /camera/image_raw --duration 2 2>/dev/null | tail -1 | awk '{print $2}' | cut -d'.' -f1)

        # Test detection performance if available
        local detection_test="{}"
        if ros2 topic info /vision/detections &>/dev/null; then
            detection_test="{\"detection_topic_available\": true}"
        fi

        cat << EOF
{
  "camera_fps": $fps,
  "detection_test": $detection_test
}
EOF
    else
        cat << EOF
{
  "camera_available": false,
  "error": "Camera not publishing"
}
EOF
    fi
}

# Function to test safety performance
test_safety_performance() {
    print_info "Testing safety system performance..."

    local start_time=$(date +%s%N)

    # Test safety dashboard response
    if ros2 topic echo /safety/dashboard_status --once --timeout 2 &>/dev/null; then
        local dashboard_response=true
    else
        local dashboard_response=false
    fi

    # Test emergency stop service response time
    local estop_start=$(date +%s%N)
    if ros2 service call /state_machine/software_estop autonomy_interfaces/srv/SoftwareEstop "
operator_id: 'benchmark_test'
reason: 'performance_test'
acknowledge_criticality: true
force_immediate: false
" --timeout 5 &>/dev/null; then
        local estop_end=$(date +%s%N)
        local estop_latency=$(( (estop_end - estop_start) / 1000000 ))  # milliseconds
        local estop_available=true
    else
        local estop_latency=0
        local estop_available=false
    fi

    local end_time=$(date +%s%N)
    local total_duration=$(( (end_time - start_time) / 1000000 ))  # milliseconds

    cat << EOF
{
  "dashboard_responsive": $dashboard_response,
  "estop_available": $estop_available,
  "estop_latency_ms": $estop_latency,
  "total_test_duration_ms": $total_duration
}
EOF
}

# Function to analyze results
analyze_results() {
    print_header "Performance Analysis"

    # Read the results file
    if [ ! -f "$OUTPUT_FILE" ]; then
        print_error "Results file not found: $OUTPUT_FILE"
        return 1
    fi

    # Extract key metrics using jq if available, otherwise basic analysis
    if command -v jq &> /dev/null; then
        echo "📊 Key Performance Metrics:"
        echo ""

        # CPU Usage
        local avg_cpu=$(jq '.samples[].system.cpu.total_used' "$OUTPUT_FILE" 2>/dev/null | awk '{sum+=$1} END {print sum/NR}' || echo "0")
        printf "CPU Usage: %.1f%%\n" "$avg_cpu"

        # Memory Usage
        local avg_mem=$(jq '.samples[].system.memory.usage_percent' "$OUTPUT_FILE" 2>/dev/null | awk '{sum+=$1} END {print sum/NR}' || echo "0")
        printf "Memory Usage: %.1f%%\n" "$avg_mem"

        # Topic Frequencies
        echo ""
        echo "Topic Frequencies:"
        jq -r '.samples[0].ros2.topics.frequencies | to_entries[] | select(.value != null) | "\(.key): \(.value)Hz"' "$OUTPUT_FILE" 2>/dev/null || echo "No frequency data"

        # Performance Test Results
        echo ""
        echo "Performance Tests:"
        jq -r '.performance_tests | to_entries[] | "\(.key): \(.value | if type == "object" then (.average_latency_ms // .goal_processing_time_ms // "completed") else . end)"' "$OUTPUT_FILE" 2>/dev/null || echo "No performance test data"

    else
        print_warning "jq not available - install for detailed analysis"
        echo "Basic analysis of results file:"
        head -20 "$OUTPUT_FILE"
    fi
}

# Function to generate report
generate_report() {
    print_header "Generating Benchmark Report"

    if [ ! -f "$OUTPUT_FILE" ]; then
        print_error "No benchmark data collected"
        return 1
    fi

    local report_file="benchmark_analysis_$(date +%Y%m%d_%H%M%S).txt"

    cat > "$report_file" << EOF
URC 2026 Autonomy System Benchmark Report
=========================================

Benchmark Configuration:
- Duration: ${DURATION} seconds
- Timestamp: $(date)
- Results File: $OUTPUT_FILE

System Performance Summary:
$(analyze_results 2>/dev/null | grep -E "(CPU|Memory|Topic|Performance)" | head -10)

Recommendations:
- CPU usage should be < 80% during normal operation
- Memory usage should be < 90% with buffer for spikes
- Critical topics should publish at expected frequencies
- Service response times should be < 1000ms

For detailed analysis, install jq and re-run analysis:
  sudo apt install jq
  ./scripts/analyze_benchmark.sh $OUTPUT_FILE

Raw data available in: $OUTPUT_FILE
EOF

    print_success "Benchmark report saved: $report_file"
}

# Function to show usage
usage() {
    echo "Usage: $0 [duration]"
    echo ""
    echo "System Performance Benchmarking for URC 2026 Autonomy"
    echo ""
    echo "Arguments:"
    echo "  duration  Benchmark duration in seconds (default: 30)"
    echo ""
    echo "Description:"
    echo "  This script measures system performance including:"
    echo "  - CPU and memory usage"
    echo "  - ROS2 topic frequencies"
    echo "  - Service response times"
    echo "  - Subsystem performance tests"
    echo ""
    echo "Requirements:"
    echo "  - Running autonomy system (see test_full_system.sh)"
    echo "  - jq (optional, for detailed analysis)"
    echo ""
    echo "Output:"
    echo "  - JSON results file with timestamp"
    echo "  - Text analysis report"
    echo "  - Console performance summary"
}

# Main execution
main() {
    # Parse arguments
    while [[ $# -gt 0 ]]; do
        case $1 in
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
                if [[ $1 =~ ^[0-9]+$ ]]; then
                    DURATION=$1
                else
                    print_error "Invalid duration: $1"
                    usage
                    exit 1
                fi
                shift
                ;;
        esac
    done

    echo "⚡ URC 2026 System Performance Benchmark"
    echo "======================================="
    echo "Duration: ${DURATION} seconds"
    echo "Output: $OUTPUT_FILE"
    echo ""

    check_prerequisites

    # Initialize results file
    echo '{"benchmark": {"duration_seconds": '$DURATION', "start_time": "'$(date)'"}, "samples": [], "performance_tests": {}}' | jq . > "$OUTPUT_FILE" 2>/dev/null || echo '{"benchmark": {"duration_seconds": '$DURATION', "start_time": "'$(date)'"}, "samples": [], "performance_tests": {}}' > "$OUTPUT_FILE"

    print_info "Starting ${DURATION}-second benchmark..."

    local start_time=$(date +%s)
    local sample_count=0

    # Collect samples during benchmark period
    while [ $(( $(date +%s) - start_time )) -lt $DURATION ]; do
        # Collect system metrics
        local system_metrics=$(collect_system_metrics)
        local ros2_metrics=$(collect_ros2_metrics)

        # Combine metrics
        local sample="{\"timestamp\": $(date +%s), $system_metrics, $ros2_metrics}"

        # Add to results (using jq if available, otherwise append)
        if command -v jq &> /dev/null; then
            jq --argjson sample "$sample" '.samples += [$sample]' "$OUTPUT_FILE" > "${OUTPUT_FILE}.tmp" && mv "${OUTPUT_FILE}.tmp" "$OUTPUT_FILE"
        else
            # Fallback: just append (will create invalid JSON, but better than nothing)
            echo "$sample" >> "${OUTPUT_FILE}.samples"
        fi

        ((sample_count++))
        sleep 2
    done

    print_success "Collected $sample_count performance samples"

    # Run performance tests
    local perf_tests=$(run_performance_tests)

    # Add performance tests to results
    if command -v jq &> /dev/null; then
        jq --argjson perf "$perf_tests" '.performance_tests = $perf' "$OUTPUT_FILE" > "${OUTPUT_FILE}.tmp" && mv "${OUTPUT_FILE}.tmp" "$OUTPUT_FILE"
    fi

    # Analyze and report
    analyze_results
    generate_report

    print_success "Benchmark completed! Results saved to $OUTPUT_FILE"
}

# Run main function
main "$@"
