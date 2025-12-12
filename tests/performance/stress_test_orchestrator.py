#!/usr/bin/env python3
"""
URC 2026 Communication Stress Test Orchestrator

Orchestrates and monitors comprehensive stress testing of the rover's communication
systems under conditions harsher than real-world scenarios.
"""

import argparse
import json
import os
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

# Add project paths
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..'))


class StressTestOrchestrator:
    """Orchestrates comprehensive communication stress testing."""

    def __init__(self, output_dir: str = "stress_test_results"):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        self.results = {}
        self.start_time = None

    def run_full_stress_test_suite(self) -> Dict:
        """Run the complete stress test suite."""

        print("🚀 URC 2026 Complete Communication Stress Test Suite")
        print("=" * 65)
        print("Testing communication systems under harsher-than-real-world conditions")
        print()

        self.start_time = time.time()
        self.results = {
            'test_metadata': {
                'start_time': datetime.now().isoformat(),
                'hostname': os.uname().nodename,
                'python_version': sys.version,
                'test_suite_version': '1.0.0'
            },
            'network_tests': {},
            'can_tests': {},
            'movement_tests': {},
            'integrated_tests': {},
            'system_health': {},
            'recommendations': []
        }

        try:
            # Run individual stress tests
            print("📡 PHASE 1: Network Communication Stress Tests")
            print("-" * 50)
            self._run_network_stress_tests()

            print("\n🔧 PHASE 2: CAN Bus Communication Stress Tests")
            print("-" * 50)
            self._run_can_stress_tests()

            print("\n🎮 PHASE 3: Movement Control Stress Tests")
            print("-" * 50)
            self._run_movement_stress_tests()

            print("\n🔗 PHASE 4: Integrated System Stress Tests")
            print("-" * 50)
            self._run_integrated_stress_tests()

            # Generate comprehensive analysis
            print("\n📊 PHASE 5: Comprehensive Analysis & Recommendations")
            print("-" * 50)
            self._generate_comprehensive_analysis()

            # Save results
            self._save_results()

            return self.results

        except Exception as e:
            print(f"❌ Stress test suite failed: {e}")
            self.results['error'] = str(e)
            return self.results

        finally:
            self.results['test_metadata']['end_time'] = datetime.now().isoformat()
            self.results['test_metadata']['duration_seconds'] = time.time() - self.start_time

    def _run_network_stress_tests(self):
        """Run network communication stress tests."""
        from stress_test_network_communication import run_comprehensive_network_stress_test

        try:
            print("   Running network stress tests...")
            network_results = run_comprehensive_network_stress_test()
            self.results['network_tests'] = network_results
            print("   ✅ Network stress tests completed")
        except Exception as e:
            print(f"   ❌ Network stress tests failed: {e}")
            self.results['network_tests'] = {'error': str(e)}

    def _run_can_stress_tests(self):
        """Run CAN bus communication stress tests."""
        from stress_test_can_communication import run_comprehensive_can_stress_test

        try:
            print("   Running CAN bus stress tests...")
            can_results = run_comprehensive_can_stress_test()
            self.results['can_tests'] = can_results
            print("   ✅ CAN bus stress tests completed")
        except Exception as e:
            print(f"   ❌ CAN bus stress tests failed: {e}")
            self.results['can_tests'] = {'error': str(e)}

    def _run_movement_stress_tests(self):
        """Run movement control stress tests."""
        from stress_test_movement_control import run_comprehensive_movement_stress_test

        try:
            print("   Running movement control stress tests...")
            movement_results = run_comprehensive_movement_stress_test()
            self.results['movement_tests'] = movement_results
            print("   ✅ Movement control stress tests completed")
        except Exception as e:
            print(f"   ❌ Movement control stress tests failed: {e}")
            self.results['movement_tests'] = {'error': str(e)}

    def _run_integrated_stress_tests(self):
        """Run integrated system stress tests."""
        from stress_test_integrated_system import run_integrated_stress_test_suite

        try:
            print("   Running integrated system stress tests...")
            integrated_results = run_integrated_stress_test_suite()
            self.results['integrated_tests'] = integrated_results
            print("   ✅ Integrated system stress tests completed")
        except Exception as e:
            print(f"   ❌ Integrated system stress tests failed: {e}")
            self.results['integrated_tests'] = {'error': str(e)}

    def _generate_comprehensive_analysis(self):
        """Generate comprehensive analysis and recommendations."""

        analysis = {
            'overall_system_health': self._calculate_overall_health(),
            'critical_findings': [],
            'performance_benchmarks': {},
            'failure_modes': [],
            'improvement_priorities': []
        }

        # Analyze network performance
        network_analysis = self._analyze_network_performance()
        analysis.update(network_analysis)

        # Analyze CAN performance
        can_analysis = self._analyze_can_performance()
        analysis.update(can_analysis)

        # Analyze movement performance
        movement_analysis = self._analyze_movement_performance()
        analysis.update(movement_analysis)

        # Analyze integrated performance
        integrated_analysis = self._analyze_integrated_performance()
        analysis.update(integrated_analysis)

        # Generate recommendations
        recommendations = self._generate_recommendations(analysis)
        analysis['recommendations'] = recommendations

        self.results['comprehensive_analysis'] = analysis

        # Display summary
        self._display_analysis_summary(analysis)

    def _calculate_overall_health(self) -> float:
        """Calculate overall system health score (0-100)."""

        health_scores = []

        # Network health
        if 'extreme' in self.results.get('network_tests', {}):
            extreme_network = self.results['network_tests']['extreme']
            # Lower score for higher latency
            network_health = max(0, 100 - (extreme_network.get('avg_latency_ms', 0) / 5))
            health_scores.append(network_health)

        # CAN health
        if 'extreme' in self.results.get('can_tests', {}):
            extreme_can = self.results['can_tests']['extreme']
            can_health = extreme_can.get('bus_availability_percent', 0)
            health_scores.append(can_health)

        # Movement health
        if 'extreme' in self.results.get('movement_tests', {}):
            extreme_movement = self.results['movement_tests']['extreme']
            movement_health = extreme_movement.get('command_success_rate', 0)
            health_scores.append(movement_health)

        # Integrated health
        if 'extreme' in self.results.get('integrated_tests', {}):
            extreme_integrated = self.results['integrated_tests']['extreme']
            if 'overall_health_score' in extreme_integrated:
                integrated_health = extreme_integrated['overall_health_score']
                health_scores.append(integrated_health)

        return sum(health_scores) / len(health_scores) if health_scores else 0

    def _analyze_network_performance(self) -> Dict:
        """Analyze network performance across all tests."""

        analysis = {'network_findings': []}

        network_results = self.results.get('network_tests', {})

        for level, result in network_results.items():
            if isinstance(result, dict) and 'avg_latency_ms' in result:
                latency = result['avg_latency_ms']
                packet_loss = result.get('packet_loss_rate', 0)

                if latency > 200:
                    analysis['network_findings'].append({
                        'level': level,
                        'issue': 'High latency',
                        'value': ".1f",
                        'severity': 'high' if latency > 500 else 'medium'
                    })

                if packet_loss > 10:
                    analysis['network_findings'].append({
                        'level': level,
                        'issue': 'High packet loss',
                        'value': ".1f",
                        'severity': 'high'
                    })

        return analysis

    def _analyze_can_performance(self) -> Dict:
        """Analyze CAN bus performance across all tests."""

        analysis = {'can_findings': []}

        can_results = self.results.get('can_tests', {})

        for level, result in can_results.items():
            if isinstance(result, dict) and 'bus_availability_percent' in result:
                availability = result['bus_availability_percent']
                error_rate = result.get('error_rate_percent', 0)

                if availability < 90:
                    analysis['can_findings'].append({
                        'level': level,
                        'issue': 'Low bus availability',
                        'value': ".1f",
                        'severity': 'high' if availability < 80 else 'medium'
                    })

                if error_rate > 15:
                    analysis['can_findings'].append({
                        'level': level,
                        'issue': 'High error rate',
                        'value': ".1f",
                        'severity': 'high'
                    })

        return analysis

    def _analyze_movement_performance(self) -> Dict:
        """Analyze movement control performance across all tests."""

        analysis = {'movement_findings': []}

        movement_results = self.results.get('movement_tests', {})

        for level, result in movement_results.items():
            if isinstance(result, dict) and 'command_success_rate' in result:
                success_rate = result['command_success_rate']
                conflicts = result.get('command_conflicts', 0)
                latency = result.get('avg_response_latency_ms', 0)

                if success_rate < 85:
                    analysis['movement_findings'].append({
                        'level': level,
                        'issue': 'Low command success rate',
                        'value': ".1f",
                        'severity': 'high' if success_rate < 70 else 'medium'
                    })

                if conflicts > 20:
                    analysis['movement_findings'].append({
                        'level': level,
                        'issue': 'High command conflicts',
                        'value': conflicts,
                        'severity': 'medium'
                    })

                if latency > 30:
                    analysis['movement_findings'].append({
                        'level': level,
                        'issue': 'High response latency',
                        'value': ".1f",
                        'severity': 'medium'
                    })

        return analysis

    def _analyze_integrated_performance(self) -> Dict:
        """Analyze integrated system performance."""

        analysis = {'integrated_findings': []}

        integrated_results = self.results.get('integrated_tests', {})

        for level, result in integrated_results.items():
            if isinstance(result, dict) and 'overall_health_score' in result:
                health_score = result['overall_health_score']

                if health_score < 70:
                    analysis['integrated_findings'].append({
                        'level': level,
                        'issue': 'Low integrated health score',
                        'value': ".1f",
                        'severity': 'high' if health_score < 50 else 'medium'
                    })

        return analysis

    def _generate_recommendations(self, analysis: Dict) -> List[str]:
        """Generate improvement recommendations based on analysis."""

        recommendations = []

        # Network recommendations
        network_findings = analysis.get('network_findings', [])
        if any(f['severity'] == 'high' for f in network_findings):
            recommendations.extend([
                "Implement network fault tolerance mechanisms",
                "Add message queuing and retransmission for critical topics",
                "Consider redundant network interfaces",
                "Implement adaptive QoS based on network conditions"
            ])

        # CAN recommendations
        can_findings = analysis.get('can_findings', [])
        if any(f['severity'] == 'high' for f in can_findings):
            recommendations.extend([
                "Add CAN bus redundancy with automatic failover",
                "Implement message prioritization and arbitration improvements",
                "Add electrical noise filtering and signal conditioning",
                "Implement bus load monitoring and rate limiting"
            ])

        # Movement recommendations
        movement_findings = analysis.get('movement_findings', [])
        if any(f['severity'] == 'high' for f in movement_findings):
            recommendations.extend([
                "Implement command validation and safety limits",
                "Add command smoothing to prevent rapid direction changes",
                "Implement emergency stop precedence in command processing",
                "Add redundant command channels with conflict resolution"
            ])

        # General recommendations
        overall_health = analysis.get('overall_system_health', 0)
        if overall_health < 60:
            recommendations.extend([
                "Comprehensive system redesign required for harsh environments",
                "Implement extensive fault tolerance and recovery mechanisms",
                "Add comprehensive system monitoring and health checking",
                "Consider architectural changes for better fault isolation"
            ])
        elif overall_health < 80:
            recommendations.extend([
                "Additional fault tolerance measures recommended",
                "Improve error detection and recovery mechanisms",
                "Enhance system monitoring capabilities",
                "Consider load balancing and resource optimization"
            ])
        else:
            recommendations.append("System demonstrates good resilience - focus on monitoring and maintenance")

        return recommendations

    def _display_analysis_summary(self, analysis: Dict):
        """Display comprehensive analysis summary."""

        print("
🎯 COMPREHENSIVE STRESS TEST ANALYSIS")
        print("=" * 45)

        overall_health = analysis.get('overall_system_health', 0)
        print(".1f"
        if overall_health > 80:
            status = "✅ EXCELLENT - System handles extreme stress well"
        elif overall_health > 60:
            status = "⚠️ GOOD - Some degradation under extreme stress"
        else:
            status = "❌ POOR - Significant issues under stress"

        print(f"   Status: {status}")

        # Component breakdown
        print("
   Critical Findings:")

        for finding_type in ['network_findings', 'can_findings', 'movement_findings', 'integrated_findings']:
            findings = analysis.get(finding_type, [])
            if findings:
                category_name = finding_type.replace('_findings', '').title()
                print(f"   {category_name}:")
                for finding in findings:
                    severity_icon = "🔴" if finding['severity'] == 'high' else "🟡"
                    print(f"     {severity_icon} {finding['issue']}: {finding['value']}")

        # Recommendations
        print("
   Top Recommendations:")
        recommendations = analysis.get('recommendations', [])[:5]  # Top 5
        for i, rec in enumerate(recommendations, 1):
            print(f"     {i}. {rec}")

    def _save_results(self):
        """Save comprehensive test results to file."""

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"stress_test_results_{timestamp}.json"

        output_file = self.output_dir / filename

        # Convert Path objects and non-serializable types
        serializable_results = self._make_serializable(self.results)

        with open(output_file, 'w') as f:
            json.dump(serializable_results, f, indent=2, default=str)

        print(f"\n💾 Results saved to: {output_file}")

        # Generate summary report
        summary_file = self.output_dir / f"stress_test_summary_{timestamp}.txt"
        self._generate_summary_report(summary_file)

    def _make_serializable(self, obj):
        """Make object serializable for JSON."""
        if isinstance(obj, dict):
            return {key: self._make_serializable(value) for key, value in obj.items()}
        elif isinstance(obj, list):
            return [self._make_serializable(item) for item in obj]
        elif isinstance(obj, Path):
            return str(obj)
        else:
            return obj

    def _generate_summary_report(self, output_file: Path):
        """Generate a human-readable summary report."""

        with open(output_file, 'w') as f:
            f.write("URC 2026 Communication Stress Test Summary Report\n")
            f.write("=" * 55 + "\n\n")

            f.write(f"Test Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Duration: {self.results['test_metadata']['duration_seconds']:.1f} seconds\n\n")

            # Overall health
            analysis = self.results.get('comprehensive_analysis', {})
            overall_health = analysis.get('overall_system_health', 0)
            f.write(f"Overall System Health Score: {overall_health:.1f}/100\n\n")

            # Component summaries
            f.write("COMPONENT PERFORMANCE SUMMARY\n")
            f.write("-" * 35 + "\n")

            components = [
                ('Network', self.results.get('network_tests', {})),
                ('CAN Bus', self.results.get('can_tests', {})),
                ('Movement', self.results.get('movement_tests', {})),
                ('Integrated', self.results.get('integrated_tests', {}))
            ]

            for comp_name, comp_results in components:
                f.write(f"\n{comp_name} Tests:\n")
                for level, result in comp_results.items():
                    if isinstance(result, dict) and 'error' not in result:
                        f.write(f"  {level.title()}: Completed\n")
                    else:
                        f.write(f"  {level.title()}: Failed\n")

            # Critical findings
            f.write("
CRITICAL FINDINGS\n")
            f.write("-" * 20 + "\n")

            for finding_type in ['network_findings', 'can_findings', 'movement_findings', 'integrated_findings']:
                findings = analysis.get(finding_type, [])
                if findings:
                    category_name = finding_type.replace('_findings', '').title()
                    f.write(f"\n{category_name} Issues:\n")
                    for finding in findings:
                        severity = finding['severity'].upper()
                        f.write(f"  - {severity}: {finding['issue']} ({finding['value']})\n")

            # Recommendations
            f.write("
RECOMMENDATIONS\n")
            f.write("-" * 15 + "\n")

            recommendations = analysis.get('recommendations', [])
            for i, rec in enumerate(recommendations, 1):
                f.write(f"{i}. {rec}\n")

            f.write(f"\nDetailed results available in: stress_test_results_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json\n")

        print(f"📄 Summary report saved to: {output_file}")


def main():
    """Main entry point for stress test orchestration."""

    parser = argparse.ArgumentParser(description="URC 2026 Communication Stress Test Orchestrator")
    parser.add_argument(
        "--output-dir",
        default="stress_test_results",
        help="Directory to save test results"
    )
    parser.add_argument(
        "--quick-test",
        action="store_true",
        help="Run a quick test with reduced duration"
    )

    args = parser.parse_args()

    # Create orchestrator
    orchestrator = StressTestOrchestrator(args.output_dir)

    # Run the complete test suite
    results = orchestrator.run_full_stress_test_suite()

    # Exit with appropriate code
    overall_health = results.get('comprehensive_analysis', {}).get('overall_system_health', 0)
    exit_code = 0 if overall_health > 60 else 1  # Fail if health is critically low

    print(f"\n🏁 Stress test suite completed with health score: {overall_health:.1f}/100")
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
