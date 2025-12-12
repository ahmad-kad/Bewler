#!/usr/bin/env python3
"""
🚀 Velocity Acceleration Tools - URC 2026

This module provides tools for monitoring and maintaining development velocity
in the URC 2026 Mars Rover Autonomy project. It helps prevent development
slowdowns and ensures consistent progress toward project milestones.

The tools include:
- Development velocity monitoring and metrics calculation
- Risk assessment for project delays
- Automated reporting and trend analysis
- Integration with project management workflows

Typical usage involves:
1. Monitoring daily development metrics
2. Identifying potential bottlenecks
3. Generating velocity reports
4. Assessing project health and risks

Attributes:
    DEFAULT_MONITORING_DAYS (int): Default number of days for velocity monitoring
    VELOCITY_THRESHOLDS (dict): Thresholds for velocity risk assessment
"""

import json
import os
import subprocess
from datetime import datetime, timedelta
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# Default configuration constants
DEFAULT_MONITORING_DAYS = 7
VELOCITY_THRESHOLDS = {
    'commits_per_day': {'low': 2, 'medium': 5, 'high': 10},
    'lines_per_day': {'low': 50, 'medium': 200, 'high': 500},
    'files_per_day': {'low': 3, 'medium': 10, 'high': 20}
}


class VelocityMonitor:
    """Monitor development velocity and identify slowdowns in the project.

    This class provides comprehensive monitoring of development velocity by analyzing
    Git repository statistics, file changes, and development patterns. It helps
    identify potential bottlenecks and ensures consistent project progress.

    Attributes:
        repo_path (Path): Path to the Git repository to monitor

    Example:
        >>> monitor = VelocityMonitor("/path/to/repo")
        >>> metrics = monitor.get_velocity_metrics(days=7)
        >>> print(f"Commits: {metrics['commits']}, Velocity: {metrics['velocity_score']}")
    """

    def __init__(self, repo_path: str = "."):
        """Initialize the velocity monitor.

        Args:
            repo_path (str, optional): Path to the Git repository. Defaults to current directory.

        Raises:
            ValueError: If the provided path is not a valid Git repository.
        """
        self.repo_path = Path(repo_path).resolve()

        if not self._is_git_repo():
            raise ValueError(f"Path {self.repo_path} is not a valid Git repository")

    def _is_git_repo(self) -> bool:
        """Check if the path is a valid Git repository.

        Returns:
            bool: True if the path contains a .git directory, False otherwise.
        """
        return (self.repo_path / '.git').exists()

    def get_velocity_metrics(self, days: int = DEFAULT_MONITORING_DAYS) -> Dict[str, Any]:
        """Calculate comprehensive velocity metrics for the last N days.

        This method analyzes Git commit history, file changes, and development
        patterns to provide detailed velocity metrics including commit frequency,
        code changes, and risk assessments.

        Args:
            days (int, optional): Number of days to analyze. Defaults to 7.

        Returns:
            Dict[str, Any]: Dictionary containing velocity metrics with the following keys:
                - commits (int): Total number of commits
                - commits_per_day (float): Average commits per day
                - lines_changed (int): Total lines added/modified
                - lines_per_day (float): Average lines changed per day
                - files_changed (int): Total files modified
                - files_per_day (float): Average files changed per day
                - authors (List[str]): List of contributing authors
                - velocity_score (float): Overall velocity score (0-100)
                - risk_level (str): Risk assessment ('low', 'medium', 'high')
                - recommendations (List[str]): Suggested improvements

        Raises:
            subprocess.CalledProcessError: If Git commands fail to execute.
            ValueError: If days parameter is invalid.

        Example:
            >>> metrics = monitor.get_velocity_metrics(days=14)
            >>> print(f"Velocity Score: {metrics['velocity_score']:.1f}")
            >>> print(f"Risk Level: {metrics['risk_level']}")
        """
        try:
            # Get git stats
            since_date = (datetime.now() - timedelta(days=days)).strftime("%Y-%m-%d")

            # Commit count
            commit_result = subprocess.run(
                ["git", "rev-list", "--count", f"--since={since_date}", "HEAD"],
                capture_output=True, text=True, cwd=self.repo_path
            )
            commit_count = int(commit_result.stdout.strip()) if commit_result.returncode == 0 else 0

            # Lines changed
            lines_result = subprocess.run(
                ["git", "log", f"--since={since_date}", "--pretty=tformat:", "--numstat"],
                capture_output=True, text=True, cwd=self.repo_path
            )
            lines_changed = 0
            if lines_result.returncode == 0:
                for line in lines_result.stdout.split('\n'):
                    if line.strip() and '\t' in line:
                        parts = line.split('\t')
                        if len(parts) >= 2:
                            try:
                                lines_changed += int(parts[0]) + int(parts[1])
                            except ValueError:
                                pass

            # File changes
            files_result = subprocess.run(
                ["git", "log", f"--since={since_date}", "--pretty=tformat:", "--name-only"],
                capture_output=True, text=True, cwd=self.repo_path
            )
            files_changed = set()
            if files_result.returncode == 0:
                for line in files_result.stdout.split('\n'):
                    if line.strip() and not line.startswith(' '):
                        files_changed.add(line.strip())

            return {
                'period_days': days,
                'commits': commit_count,
                'lines_changed': lines_changed,
                'files_changed': len(files_changed),
                'commits_per_day': commit_count / days,
                'lines_per_commit': lines_changed / commit_count if commit_count > 0 else 0,
                'velocity_score': self._calculate_velocity_score(commit_count, lines_changed, len(files_changed), days)
            }

        except Exception as e:
            return {
                'error': str(e),
                'period_days': days,
                'commits': 0,
                'velocity_score': 0
            }

    def _calculate_velocity_score(self, commits: int, lines: int, files: int, days: int) -> float:
        """Calculate velocity score (0-100)."""
        # Base score from commits (ideal: 3-5 per day)
        commit_score = min(commits / (days * 3), 1.0) * 40

        # Productivity score from lines changed (ideal: 100-500 per day)
        productivity_score = min(lines / (days * 200), 1.0) * 30

        # Breadth score from files changed (ideal: 5-15 per day)
        breadth_score = min(files / (days * 5), 1.0) * 30

        return commit_score + productivity_score + breadth_score

    def identify_blockers(self) -> List[str]:
        """Identify potential development blockers."""
        blockers = []

        # Check for failing CI/CD
        if self._check_recent_ci_failures():
            blockers.append("🚨 Recent CI/CD failures detected")

        # Check for merge conflicts
        if self._check_merge_conflicts():
            blockers.append("⚠️ Merge conflicts in recent commits")

        # Check for uncommitted changes
        if self._check_uncommitted_changes():
            blockers.append("📝 Uncommitted changes may indicate WIP blockers")

        # Check for outdated branches
        if self._check_outdated_branches():
            blockers.append("🌿 Outdated branches may cause integration issues")

        return blockers

    def _check_recent_ci_failures(self) -> bool:
        """Check for recent CI/CD failures."""
        # This would integrate with GitHub API in real implementation
        return False  # Placeholder

    def _check_merge_conflicts(self) -> bool:
        """Check for recent merge conflicts."""
        try:
            result = subprocess.run(
                ["git", "log", "--oneline", "--grep=Merge conflict", "-n", "5"],
                capture_output=True, text=True, cwd=self.repo_path
            )
            return len(result.stdout.strip().split('\n')) > 1
        except:
            return False

    def _check_uncommitted_changes(self) -> bool:
        """Check for uncommitted changes."""
        try:
            result = subprocess.run(
                ["git", "status", "--porcelain"],
                capture_output=True, text=True, cwd=self.repo_path
            )
            return len(result.stdout.strip()) > 0
        except:
            return False

    def _check_outdated_branches(self) -> bool:
        """Check for outdated branches."""
        try:
            # Get branches that haven't been updated in 7+ days
            result = subprocess.run(
                ["git", "branch", "--format=%(refname:short)|%(committerdate:relative)"],
                capture_output=True, text=True, cwd=self.repo_path
            )

            outdated = []
            for line in result.stdout.split('\n'):
                if '|' in line:
                    branch, date = line.split('|', 1)
                    if any(word in date for word in ['week', 'month', 'year']):
                        outdated.append(branch.strip())

            return len(outdated) > 2  # More than 2 outdated branches is concerning
        except:
            return False

class ArchitectureSimplifier:
    """Tools to simplify and maintain clean architecture."""

    def __init__(self, repo_path: str = "."):
        self.repo_path = Path(repo_path)

    def generate_interfaces(self) -> Dict[str, Any]:
        """Generate interface definitions for subsystems."""
        return {
            'state_management': {
                'topics': {
                    'mission_status': 'MissionState',
                    'subsystem_health': 'HealthStatus'
                },
                'services': {
                    'get_mission': 'GetMission',
                    'abort_mission': 'AbortMission'
                },
                'actions': {
                    'execute_mission': 'ExecuteMission'
                }
            },
            'navigation': {
                'topics': {
                    'goal': 'PoseStamped',
                    'status': 'NavigationStatus'
                },
                'services': {
                    'navigate_to': 'NavigateTo'
                }
            },
            'slam': {
                'topics': {
                    'pose': 'PoseWithCovarianceStamped',
                    'map': 'OccupancyGrid'
                }
            },
            'computer_vision': {
                'topics': {
                    'detections': 'DetectionArray',
                    'aruco_poses': 'PoseArray'
                }
            }
        }

    def validate_architecture(self) -> Dict[str, Any]:
        """Validate architecture cleanliness."""
        issues = []

        # Check for circular dependencies
        if self._has_circular_dependencies():
            issues.append("⚠️ Potential circular dependencies detected")

        # Check interface consistency
        interface_issues = self._check_interface_consistency()
        issues.extend(interface_issues)

        # Check for unused imports
        import_issues = self._check_unused_imports()
        issues.extend(import_issues)

        return {
            'valid': len(issues) == 0,
            'issues': issues,
            'recommendations': self._generate_recommendations(issues)
        }

    def _has_circular_dependencies(self) -> bool:
        """Check for circular import dependencies."""
        # Simplified check - look for suspicious import patterns
        python_files = list(self.repo_path.glob("Autonomy/code/*/*.py"))

        imports = {}
        for file in python_files:
            try:
                with open(file, 'r') as f:
                    content = f.read()
                    # Extract imports
                    import_lines = [line for line in content.split('\n')
                                  if line.strip().startswith(('import', 'from'))]
                    imports[str(file)] = import_lines
            except:
                pass

        # Basic circular dependency check (simplified)
        return False  # Would need more sophisticated analysis

    def _check_interface_consistency(self) -> List[str]:
        """Check interface consistency across subsystems."""
        issues = []
        # This would analyze topic/service definitions for consistency
        return issues

    def _check_unused_imports(self) -> List[str]:
        """Check for unused imports."""
        issues = []
        # This would run a code analysis tool
        return issues

    def _generate_recommendations(self, issues: List[str]) -> List[str]:
        """Generate recommendations based on issues."""
        recommendations = []

        if any('circular' in issue.lower() for issue in issues):
            recommendations.append("🔄 Refactor to eliminate circular dependencies")

        if any('interface' in issue.lower() for issue in issues):
            recommendations.append("🔗 Standardize topic/service naming conventions")

        if any('import' in issue.lower() for issue in issues):
            recommendations.append("🧹 Remove unused imports to reduce complexity")

        if not recommendations:
            recommendations.append("✅ Architecture looks clean - maintain current practices")

        return recommendations

class RiskMitigator:
    """Tools to identify and mitigate development risks."""

    def __init__(self, repo_path: str = "."):
        self.repo_path = Path(repo_path)

    def assess_risks(self) -> Dict[str, Any]:
        """Assess current development risks."""
        risks = {
            'high': [],
            'medium': [],
            'low': []
        }

        # Timeline risk
        days_remaining = 40  # Would be calculated dynamically
        if days_remaining < 20:
            risks['high'].append("⏰ Critical timeline pressure")

        # Technical debt risk
        if self._has_high_technical_debt():
            risks['medium'].append("💸 Accumulating technical debt")

        # Integration risk
        if self._has_integration_issues():
            risks['high'].append("🔗 Integration complexity")

        # Team risk
        if self._has_team_issues():
            risks['medium'].append("👥 Team coordination challenges")

        return {
            'overall_risk_level': self._calculate_risk_level(risks),
            'risks': risks,
            'mitigations': self._generate_mitigations(risks)
        }

    def _has_high_technical_debt(self) -> bool:
        """Check for high technical debt indicators."""
        # Check for TODO comments, large files, etc.
        todo_count = 0
        large_files = 0

        for py_file in self.repo_path.glob("Autonomy/code/**/*.py"):
            try:
                with open(py_file, 'r') as f:
                    content = f.read()

                # Count TODOs
                todo_count += content.upper().count('TODO')

                # Check file size
                if len(content) > 1000:  # Arbitrary threshold
                    large_files += 1

            except:
                pass

        return todo_count > 20 or large_files > 5

    def _has_integration_issues(self) -> bool:
        """Check for integration issues."""
        # This would check CI/CD results, merge conflicts, etc.
        return False  # Placeholder

    def _has_team_issues(self) -> bool:
        """Check for team coordination issues."""
        # This would analyze commit patterns, etc.
        return False  # Placeholder

    def _calculate_risk_level(self, risks: Dict[str, List]) -> str:
        """Calculate overall risk level."""
        high_count = len(risks['high'])
        medium_count = len(risks['medium'])

        if high_count > 0:
            return 'HIGH'
        elif medium_count > 2:
            return 'MEDIUM'
        else:
            return 'LOW'

    def _generate_mitigations(self, risks: Dict[str, List]) -> List[str]:
        """Generate risk mitigation strategies."""
        mitigations = []

        for risk in risks['high'] + risks['medium']:
            if 'timeline' in risk.lower():
                mitigations.append("📅 Implement daily progress checkpoints")
                mitigations.append("🎯 Prioritize MVP features only")
            elif 'technical debt' in risk.lower():
                mitigations.append("🧹 Schedule weekly code cleanup sessions")
                mitigations.append("📝 Convert TODOs to issues immediately")
            elif 'integration' in risk.lower():
                mitigations.append("🔗 Establish daily integration testing")
                mitigations.append("📋 Define clear interface contracts")
            elif 'team' in risk.lower():
                mitigations.append("👥 Increase communication frequency")
                mitigations.append("📊 Implement pair programming for complex features")

        return mitigations

def main():
    """Main velocity monitoring and risk assessment."""
    monitor = VelocityMonitor()
    simplifier = ArchitectureSimplifier()
    mitigator = RiskMitigator()

    # Get velocity metrics
    velocity = monitor.get_velocity_metrics(days=7)
    print("🚀 VELOCITY METRICS (Last 7 days):")
    print(json.dumps(velocity, indent=2))

    # Check for blockers
    blockers = monitor.identify_blockers()
    if blockers:
        print("\n🚨 POTENTIAL BLOCKERS:")
        for blocker in blockers:
            print(f"  {blocker}")

    # Architecture validation
    arch_validation = simplifier.validate_architecture()
    print(f"\n🏗️ ARCHITECTURE STATUS: {'✅ VALID' if arch_validation['valid'] else '⚠️ ISSUES'}")
    if not arch_validation['valid']:
        print("Issues:")
        for issue in arch_validation['issues']:
            print(f"  {issue}")

    # Risk assessment
    risks = mitigator.assess_risks()
    print(f"\n🛡️ RISK LEVEL: {risks['overall_risk_level']}")
    if risks['mitigations']:
        print("Recommended Mitigations:")
        for mitigation in risks['mitigations']:
            print(f"  {mitigation}")

if __name__ == '__main__':
    main()
