#!/usr/bin/env python3
"""
📊 Daily Velocity Check

Quick daily assessment to maintain velocity and catch issues early.
Run this every morning to stay on track for the 40-day timeline.
"""

import json
import subprocess
from datetime import datetime, timedelta
from pathlib import Path
from typing import Any, Dict, List


class DailyVelocityCheck:
    """Daily velocity and risk assessment."""

    def __init__(self, repo_path: str = "."):
        self.repo_path = Path(repo_path)

    def run_daily_check(self) -> Dict[str, Any]:
        """Run complete daily velocity assessment."""
        print("📊 DAILY VELOCITY CHECK")
        print("=" * 30)

        results = {
            "date": datetime.now().strftime("%Y-%m-%d"),
            "velocity_score": self._check_velocity(),
            "blockers": self._identify_blockers(),
            "progress": self._assess_progress(),
            "recommendations": self._generate_recommendations(),
            "risk_level": "LOW",
        }

        # Calculate risk level
        if len(results["blockers"]) > 2:
            results["risk_level"] = "HIGH"
        elif len(results["blockers"]) > 0 or results["velocity_score"] < 50:
            results["risk_level"] = "MEDIUM"

        self._display_results(results)
        self._save_results(results)

        return results

    def _check_velocity(self) -> float:
        """Check current development velocity."""
        try:
            # Check commits in last 24 hours
            yesterday = (datetime.now() - timedelta(days=1)).strftime("%Y-%m-%d")

            result = subprocess.run(
                ["git", "log", "--oneline", f"--since={yesterday}"], capture_output=True, text=True, cwd=self.repo_path
            )

            commit_count = len([line for line in result.stdout.split("\n") if line.strip()])

            # Velocity score based on commits (ideal: 3-8 per day)
            if commit_count >= 8:
                return 100.0  # Excellent velocity
            elif commit_count >= 5:
                return 80.0  # Good velocity
            elif commit_count >= 3:
                return 60.0  # Acceptable velocity
            elif commit_count >= 1:
                return 40.0  # Slow but moving
            else:
                return 20.0  # No progress

        except BaseException:
            return 0.0

    def _identify_blockers(self) -> List[str]:
        """Identify current development blockers."""
        blockers = []

        # Check for CI/CD failures
        if self._check_ci_failures():
            blockers.append("🚨 CI/CD pipeline failing")

        # Check for merge conflicts
        if self._check_merge_conflicts():
            blockers.append("⚠️ Unresolved merge conflicts")

        # Check for uncommitted work
        if self._check_uncommitted_work():
            blockers.append("📝 Large amount of uncommitted work")

        # Check for outdated dependencies
        if self._check_outdated_dependencies():
            blockers.append("📦 Outdated or conflicting dependencies")

        # Check for failing tests
        if self._check_failing_tests():
            blockers.append("🧪 Failing automated tests")

        return blockers

    def _check_ci_failures(self) -> bool:
        """Check for recent CI/CD failures."""
        # In real implementation, this would check GitHub API
        return False

    def _check_merge_conflicts(self) -> bool:
        """Check for recent merge conflicts."""
        try:
            result = subprocess.run(
                ["git", "log", "--oneline", "--grep=conflict", "-n", "3"],
                capture_output=True,
                text=True,
                cwd=self.repo_path,
            )
            return "conflict" in result.stdout.lower()
        except BaseException:
            return False

    def _check_uncommitted_work(self) -> bool:
        """Check for large amounts of uncommitted work."""
        try:
            result = subprocess.run(
                ["git", "status", "--porcelain"], capture_output=True, text=True, cwd=self.repo_path
            )
            lines = len([line for line in result.stdout.split("\n") if line.strip()])
            return lines > 10  # More than 10 uncommitted files
        except BaseException:
            return False

    def _check_outdated_dependencies(self) -> bool:
        """Check for outdated dependencies."""
        # Check if requirements.txt exists and is recent
        req_file = self.repo_path / "Autonomy/requirements.txt"
        if req_file.exists():
            mtime = datetime.fromtimestamp(req_file.stat().st_mtime)
            days_old = (datetime.now() - mtime).days
            return days_old > 7  # Older than a week
        return False

    def _check_failing_tests(self) -> bool:
        """Check for failing tests."""
        # In real implementation, this would run test suite
        return False

    def _assess_progress(self) -> Dict[str, Any]:
        """Assess overall project progress."""
        # Load latest progress data
        progress_file = self.repo_path / "progress_analysis.json"
        if progress_file.exists():
            try:
                with open(progress_file, "r") as f:
                    data = json.load(f)

                total_progress = sum(p.get("overall_progress", 0) for p in data.values())
                avg_progress = total_progress / len(data) if data else 0

                return {
                    "overall_progress": avg_progress * 100,
                    "subsystems": len(data),
                    "on_track": avg_progress >= 0.7,  # 70% progress target
                }
            except BaseException:
                pass

        return {"overall_progress": 0, "subsystems": 0, "on_track": False}

    def _generate_recommendations(self) -> List[str]:
        """Generate daily recommendations."""
        recommendations = [
            "🎯 Focus on MVP completion today",
            "🔗 Test integration points early",
            "📝 Commit small, frequent changes",
            "👥 Communicate blockers immediately",
        ]

        # Add specific recommendations based on current state
        progress = self._assess_progress()
        if progress["overall_progress"] < 50:
            recommendations.append("⚡ Increase development velocity - aim for 5+ commits today")

        if not progress["on_track"]:
            recommendations.append("🎯 Prioritize critical path items over nice-to-have features")

        blockers = self._identify_blockers()
        if blockers:
            recommendations.append("🚨 Resolve blockers before implementing new features")

        return recommendations

    def _display_results(self, results: Dict[str, Any]):
        """Display results in a readable format."""
        velocity = results["velocity_score"]
        risk_level = results["risk_level"]
        progress = results["progress"]["overall_progress"]

        # Velocity indicator
        if velocity >= 80:
            vel_icon = "🚀"
            vel_status = "EXCELLENT"
        elif velocity >= 60:
            vel_icon = "✅"
            vel_status = "GOOD"
        elif velocity >= 40:
            vel_icon = "⚠️"
            vel_status = "SLOW"
        else:
            vel_icon = "🚨"
            vel_status = "CRITICAL"

        # Risk indicator
        risk_colors = {"LOW": "🟢", "MEDIUM": "🟡", "HIGH": "🔴"}
        risk_icon = risk_colors.get(risk_level, "❓")

        print(f"\n{risk_icon} RISK LEVEL: {risk_level}")
        print(f"{vel_icon} VELOCITY: {velocity:.1f}% ({vel_status})")
        print(f"📈 PROGRESS: {progress:.1f}%")

        if results["blockers"]:
            print(f"\n🚨 BLOCKERS ({len(results['blockers'])}):")
            for blocker in results["blockers"]:
                print(f"  {blocker}")

        print("\n💡 RECOMMENDATIONS:")
        for rec in results["recommendations"]:
            print(f"  {rec}")

        # Timeline warning
        days_remaining = 40  # Would be calculated
        if days_remaining < 20:
            print(f"\n⏰ CRITICAL: Only {days_remaining} days remaining!")
            print("  Focus on MVP completion and integration testing")

    def _save_results(self, results: Dict[str, Any]):
        """Save results for historical tracking."""
        history_file = self.repo_path / "data" / "velocity_history.json"

        # Load existing history
        history = []
        if history_file.exists():
            try:
                with open(history_file, "r") as f:
                    history = json.load(f)
            except BaseException:
                history = []

        # Add new results
        history.append(results)

        # Keep only last 30 days
        history = history[-30:]

        # Save
        with open(history_file, "w") as f:
            json.dump(history, f, indent=2, default=str)


def main():
    """Run daily velocity check."""
    checker = DailyVelocityCheck()
    results = checker.run_daily_check()

    # Save to environment for CI/CD
    with open("data/daily_check_results.json", "w") as f:
        json.dump(results, f, indent=2)


if __name__ == "__main__":
    main()
