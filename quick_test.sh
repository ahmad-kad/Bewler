#!/bin/bash
# Quick Start Testing Script
# Fast way to test autonomy-teleoperation integration

set -e

echo "🚀 Quick Autonomy-Teleoperation Integration Test"
echo "=" * 50

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# Setup ROS2
echo -e "${BLUE}[SETUP]${NC} Setting up ROS2 environment..."
cd /home/ubuntu/urc-machiato-2026
source /opt/ros/humble/setup.bash
export PYTHONPATH="$PWD:$PYTHONPATH"
export ROVER_ENV="production"

# Start mock teleoperation in background
echo -e "${BLUE}[START]${NC} Starting mock teleoperation data..."
python3 test_teleoperation_integration.py &
MOCK_PID=$!
sleep 3

# Verify topics
echo -e "${BLUE}[CHECK]${NC} Verifying ROS2 topics..."
TOPICS=$(ros2 topic list 2>/dev/null)
TELEOP_TOPICS=(
    "/teleoperation/joint_states"
    "/teleoperation/chassis_velocity"
    "/teleoperation/motor_temperatures"
    "/teleoperation/system_status"
)

FOUND_COUNT=0
for topic in "${TELEOP_TOPICS[@]}"; do
    if echo "$TOPICS" | grep -q "$topic"; then
        echo -e "${GREEN}✓${NC} $topic"
        ((FOUND_COUNT++))
    else
        echo -e "${RED}✗${NC} $topic"
    fi
done

if [[ $FOUND_COUNT -eq 4 ]]; then
    echo -e "${GREEN}[SUCCESS]${NC} All teleoperation topics active"
else
    echo -e "${RED}[ERROR]${NC} Missing teleoperation topics"
    kill $MOCK_PID 2>/dev/null || true
    exit 1
fi

# Start autonomy
echo -e "${BLUE}[START]${NC} Starting autonomy mission executor..."
ros2 run missions mission_executor &
AUTONOMY_PID=$!
sleep 3

# Quick verification
echo -e "${BLUE}[TEST]${NC} Quick integration verification..."

# Check nodes
NODES=$(ros2 node list 2>/dev/null)
if echo "$NODES" | grep -q "simple_mission_executor"; then
    echo -e "${GREEN}✓${NC} Autonomy node running"
else
    echo -e "${RED}✗${NC} Autonomy node not found"
fi

# Check topic rates
echo -e "${BLUE}[RATE]${NC} Checking topic publishing rate..."
RATE_OUTPUT=$(timeout 3 ros2 topic hz /teleoperation/joint_states 2>/dev/null || true)
if echo "$RATE_OUTPUT" | grep -q "average rate:"; then
    RATE=$(echo "$RATE_OUTPUT" | grep "average rate:" | head -1 | sed 's/.*average rate: \([0-9.]*\).*/\1/')
    echo -e "${GREEN}✓${NC} Publishing rate: ${RATE} Hz"
else
    echo -e "${YELLOW}⚠${NC} Could not determine rate"
fi

echo
echo -e "${GREEN}[READY]${NC} Integration test environment active!"
echo
echo "🎮 Testing Commands:"
echo "==================="
echo "• Monitor data:     ros2 topic echo /teleoperation/joint_states"
echo "• Check autonomy:   ros2 node list"
echo "• View graph:       ros2 run rqt_graph rqt_graph"
echo "• Send mission:     ros2 topic pub /mission/commands std_msgs/String 'data: \"test\"'"
echo
echo "🛑 To stop: Ctrl+C or run 'pkill -f \"test_teleoperation_integration\" && pkill -f \"mission_executor\"'"
echo
echo "Press Ctrl+C to exit and cleanup..."

# Wait for user
trap 'echo -e "\n${BLUE}[CLEANUP]${NC} Stopping test processes..."; kill $MOCK_PID $AUTONOMY_PID 2>/dev/null || true; exit 0' INT TERM

# Keep running
while true; do
    sleep 1

    # Check if processes are still alive
    if ! kill -0 $MOCK_PID 2>/dev/null; then
        echo -e "${RED}[ERROR]${NC} Mock teleoperation stopped unexpectedly"
        kill $AUTONOMY_PID 2>/dev/null || true
        exit 1
    fi

    if ! kill -0 $AUTONOMY_PID 2>/dev/null; then
        echo -e "${RED}[ERROR]${NC} Autonomy stopped unexpectedly"
        kill $MOCK_PID 2>/dev/null || true
        exit 1
    fi
done
