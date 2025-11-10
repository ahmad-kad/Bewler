#!/usr/bin/env python3
"""
Race Condition Test for State Machine.

Tests for concurrent access issues and state corruption.
"""

import threading
import time
import sys
import os

# Add the state machine to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'autonomy_state_machine'))

from states import SystemState, AutonomousSubstate


class MockStateMachine:
    """Mock state machine to test race conditions."""

    def __init__(self):
        self.current_state = SystemState.IDLE
        self.transition_count = 0
        self.errors = []
        # No lock - this will demonstrate race conditions

    def unsafe_transition(self, to_state, thread_id):
        """Unsafe transition method that can cause race conditions."""
        # Simulate some processing time
        time.sleep(0.001)

        from_state = self.current_state
        self.current_state = to_state
        self.transition_count += 1

        # Simulate validation
        time.sleep(0.001)

        # This could be corrupted by concurrent access
        if str(from_state) == str(to_state):
            self.errors.append(f"Thread {thread_id}: Invalid transition {from_state} -> {to_state}")

        return True


def test_race_conditions():
    """Test for race conditions in state transitions."""
    print("Testing race conditions in state machine...")

    state_machine = MockStateMachine()
    errors_found = []

    def worker_thread(thread_id):
        """Worker thread that performs state transitions."""
        for i in range(100):
            try:
                # Try various transitions
                if i % 4 == 0:
                    state_machine.unsafe_transition(SystemState.IDLE, thread_id)
                elif i % 4 == 1:
                    state_machine.unsafe_transition(SystemState.AUTONOMOUS, thread_id)
                elif i % 4 == 2:
                    state_machine.unsafe_transition(SystemState.SAFETY, thread_id)
                else:
                    state_machine.unsafe_transition(SystemState.TELEOPERATION, thread_id)
            except Exception as e:
                errors_found.append(f"Thread {thread_id}: {str(e)}")

    # Create multiple threads
    threads = []
    for i in range(10):
        t = threading.Thread(target=worker_thread, args=(i,))
        threads.append(t)

    # Start all threads
    start_time = time.time()
    for t in threads:
        t.start()

    # Wait for completion
    for t in threads:
        t.join()

    end_time = time.time()

    print("Race condition test completed:")
    print(f"- Duration: {end_time - start_time:.2f} seconds")
    print(f"- Total transitions: {state_machine.transition_count}")
    print(f"- State machine errors: {len(state_machine.errors)}")
    print(f"- Thread errors: {len(errors_found)}")

    if state_machine.errors:
        print("\nState machine errors found:")
        for error in state_machine.errors[:5]:  # Show first 5
            print(f"  {error}")

    if errors_found:
        print("\nThread errors found:")
        for error in errors_found[:5]:  # Show first 5
            print(f"  {error}")

    # This test will likely show race conditions without proper locking
    total_errors = len(state_machine.errors) + len(errors_found)
    if total_errors > 0:
        print(f"\n❌ RACE CONDITIONS DETECTED: {total_errors} errors found")
        print("The state machine is not thread-safe!")
        return False
    else:
        print("\n✅ No race conditions detected (this run)")
        print("Note: Race conditions may still exist but didn't manifest in this test.")
        return True


if __name__ == "__main__":
    test_race_conditions()

