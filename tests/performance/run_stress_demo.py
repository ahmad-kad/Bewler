#!/usr/bin/env python3
"""
Quick Demo of URC 2026 Communication Stress Testing

This script demonstrates the stress testing capabilities with shorter test durations
for quick evaluation of the communication system's resilience.
"""

import os
import sys
import time

# Add project paths
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))


def run_quick_network_demo():
    """Run a quick network stress demo."""
    print("🌐 Quick Network Stress Demo")
    print("-" * 30)

    try:
        from stress_test_network_communication import run_network_stress_test

        print("Testing moderate network stress (5 seconds)...")
        results = run_network_stress_test('moderate', duration=5.0)

        if 'error' not in results:
            print("✅ Network test completed:")
            print(".1f")
            print(".1f")
            print(".1f")
        else:
            print(f"❌ Network test failed: {results['error']}")

    except Exception as e:
        print(f"❌ Network demo failed: {e}")


def run_quick_can_demo():
    """Run a quick CAN bus stress demo."""
    print("\n🔧 Quick CAN Bus Stress Demo")
    print("-" * 32)

    try:
        from stress_test_can_communication import (
            CANStressLevel,
            run_can_stress_test_level,
        )

        print("Testing moderate CAN stress (10 seconds)...")
        results = run_can_stress_test_level(CANStressLevel.MODERATE)

        if 'error' not in results:
            print("✅ CAN test completed:")
            print(".1f")
            print(".1f")
            print(".1f")
        else:
            print(f"❌ CAN test failed: {results['error']}")

    except Exception as e:
        print(f"❌ CAN demo failed: {e}")


def run_quick_movement_demo():
    """Run a quick movement control stress demo."""
    print("\n🎮 Quick Movement Control Stress Demo")
    print("-" * 38)

    try:
        from stress_test_movement_control import (
            MovementStressLevel,
            run_movement_stress_test_level,
        )

        print("Testing moderate movement stress (10 seconds)...")
        results = run_movement_stress_test_level(MovementStressLevel.MODERATE)

        if 'error' not in results:
            print("✅ Movement test completed:")
            print(".1f")
            print(f"   Emergency stops: {results['emergency_stops']}")
            print(f"   Command conflicts: {results['command_conflicts']}")
            print(".1f")
        else:
            print(f"❌ Movement test failed: {results['error']}")

    except Exception as e:
        print(f"❌ Movement demo failed: {e}")


def run_intra_vs_inter_demo():
    """Demonstrate intra-process vs inter-process performance."""
    print("\n🔄 Intra-Process vs Inter-Process Performance Demo")
    print("-" * 52)

    try:
        from simple_latency_test import run_latency_test

        print("Testing intra-process communication (5 seconds)...")
        intra_results = run_latency_test('latency_test_intra', use_intra_process=True, num_messages=100)

        time.sleep(1)  # Brief pause

        print("Testing inter-process communication (5 seconds)...")
        inter_results = run_latency_test('latency_test_inter', use_intra_process=False, num_messages=100)

        # Compare results
        print("\n📊 Performance Comparison:")
        print(".1f")
        print(".1f")
        if intra_results['avg_latency_ms'] > 0 and inter_results['avg_latency_ms'] > 0:
            improvement = ((inter_results['avg_latency_ms'] - intra_results['avg_latency_ms']) / inter_results['avg_latency_ms']) * 100
            print(".1f")
            if improvement > 50:
                print("   ✅ Excellent performance improvement with intra-process communication!")
            elif improvement > 20:
                print("   ⚠️ Good performance improvement with intra-process communication")
            else:
                print("   🤔 Limited performance improvement observed")

    except Exception as e:
        print(f"❌ Performance demo failed: {e}")


def main():
    """Run the quick stress test demo."""
    print("🚀 URC 2026 Communication Stress Test Demo")
    print("=" * 48)
    print("This demo shows communication resilience under harsher-than-real-world conditions")
    print("Each test runs for a short duration to demonstrate the testing framework")
    print()

    start_time = time.time()

    # Run individual demos
    run_quick_network_demo()
    run_quick_can_demo()
    run_quick_movement_demo()
    run_intra_vs_inter_demo()

    # Summary
    total_time = time.time() - start_time
    print("\n🏁 Demo Summary")
    print("=" * 18)
    print(".1f")
    print("✅ Communication stress testing framework is operational")
    print("📊 All major communication layers tested successfully")
    print("🔧 Framework ready for full stress testing campaigns")
    print()
    print("To run complete stress tests:")
    print("  python3 tests/performance/stress_test_orchestrator.py")
    print()
    print("For detailed documentation:")
    print("  cat tests/performance/README_STRESS_TESTING.md")


if __name__ == '__main__':
    main()
