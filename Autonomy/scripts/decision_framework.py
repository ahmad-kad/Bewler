#!/usr/bin/env python3
"""
🎯 Decision Framework for Velocity

Prevent backtracking by making fast, good-enough decisions.
"""

import json
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List


class DecisionFramework:
    """Framework for fast, reversible decisions."""

    def __init__(self, repo_path: str = "."):
        self.repo_path = Path(repo_path)
        self.decisions_file = self.repo_path / "decisions.json"
        self.load_decisions()

    def load_decisions(self):
        """Load existing decisions."""
        if self.decisions_file.exists():
            with open(self.decisions_file, "r") as f:
                self.decisions = json.load(f)
        else:
            self.decisions = {}

    def save_decisions(self):
        """Save decisions to file."""
        with open(self.decisions_file, "w") as f:
            json.dump(self.decisions, f, indent=2, default=str)

    def make_decision(self, topic: str, options: List[str], context: str = "") -> Dict[str, Any]:
        """Make a decision using the framework."""

        print(f"🎯 DECISION: {topic}")
        if context:
            print(f"Context: {context}")
        print(f"Options: {', '.join(options)}")

        # Quick evaluation (spend max 15 minutes)
        evaluation = self._quick_evaluation(options)

        # Choose winner
        chosen_option = evaluation["recommended"]
        confidence = evaluation["confidence"]

        # Create fallback plan
        fallback = self._create_fallback_plan(chosen_option, options)

        # Set review date
        review_date = self._calculate_review_date(confidence)

        decision_record = {
            "topic": topic,
            "chosen_option": chosen_option,
            "alternatives": [opt for opt in options if opt != chosen_option],
            "confidence": confidence,
            "evaluation": evaluation,
            "fallback": fallback,
            "review_date": review_date,
            "timestamp": datetime.now(),
            "status": "active",
        }

        # Store decision
        self.decisions[topic] = decision_record
        self.save_decisions()

        print(f"✅ DECIDED: {chosen_option} (Confidence: {confidence}%)")
        print(f"🔄 Fallback: {fallback}")
        print(f"📅 Review: {review_date}")

        return decision_record

    def _quick_evaluation(self, options: List[str]) -> Dict[str, Any]:
        """Quick evaluation of options (max 10 minutes)."""

        # Simplified evaluation - in practice, this would involve team input
        scores = {}

        for option in options:
            # Simple scoring based on heuristics
            score = 0

            # Prefer simpler options
            if len(option) < 50:  # Shorter = simpler
                score += 20

            # Prefer established technologies
            established_terms = ["ros2", "python", "opencv", "gps", "imu"]
            if any(term in option.lower() for term in established_terms):
                score += 30

            # Prefer options that reduce dependencies
            if "simple" in option.lower() or "minimal" in option.lower():
                score += 25

            # Prefer options with clear implementation path
            if "standard" in option.lower() or "common" in option.lower():
                score += 15

            scores[option] = min(score, 100)  # Cap at 100

        # Choose highest scoring option
        recommended = max(scores, key=scores.get)
        confidence = scores[recommended]

        return {
            "recommended": recommended,
            "confidence": confidence,
            "scores": scores,
            "reasoning": f"Selected based on simplicity and established technology preferences",
        }

    def _create_fallback_plan(self, chosen: str, all_options: List[str]) -> str:
        """Create a fallback plan for the decision."""
        alternatives = [opt for opt in all_options if opt != chosen]

        if alternatives:
            return f"If {chosen} fails, switch to: {alternatives[0]}"
        else:
            return "No fallback available - decision is critical path"

    def _calculate_review_date(self, confidence: float) -> str:
        """Calculate when to review this decision."""
        days = 7  # Default weekly review

        if confidence < 50:
            days = 3  # Review in 3 days if low confidence
        elif confidence < 70:
            days = 7  # Review weekly if medium confidence
        else:
            days = 14  # Review bi-weekly if high confidence

        review_date = datetime.now() + timedelta(days=days)
        return review_date.strftime("%Y-%m-%d")

    def review_decisions(self) -> List[str]:
        """Review decisions that need attention."""
        today = datetime.now()
        needs_review = []

        for topic, decision in self.decisions.items():
            if decision["status"] == "active":
                review_date = datetime.fromisoformat(decision["review_date"])
                if today >= review_date:
                    needs_review.append(topic)

        return needs_review

    def close_decision(self, topic: str, outcome: str):
        """Close a decision with final outcome."""
        if topic in self.decisions:
            self.decisions[topic]["status"] = "closed"
            self.decisions[topic]["final_outcome"] = outcome
            self.decisions[topic]["closed_date"] = datetime.now()
            self.save_decisions()


# Pre-defined common decisions for autonomy project
COMMON_DECISIONS = {
    "architecture": {
        "topic": "System Architecture Pattern",
        "options": [
            "Centralized state management with pub/sub",
            "Distributed microservices with ROS 2",
            "Hierarchical control with behavior trees",
        ],
        "context": "Need to minimize coupling while maintaining real-time performance",
    },
    "communication": {
        "topic": "Inter-subsystem Communication",
        "options": [
            "ROS 2 topics only (simplest)",
            "Topics + services for requests",
            "Full ROS 2 actions for complex operations",
        ],
        "context": "Balance simplicity with functionality",
    },
    "sensor_fusion": {
        "topic": "SLAM Sensor Fusion Approach",
        "options": ["Basic complementary filter", "Extended Kalman Filter (EKF)", "UKF with camera integration"],
        "context": "GNSS + IMU + Lidar fusion for reliable localization",
    },
    "computer_vision": {
        "topic": "Object Detection Framework",
        "options": [
            "YOLOv5 (fast, good accuracy)",
            "OpenCV Haar cascades (simple, fast)",
            "Custom CNN (flexible, complex)",
        ],
        "context": "Detect competition objects reliably and quickly",
    },
    "path_planning": {
        "topic": "Navigation Planning Algorithm",
        "options": [
            "A* with terrain costs (reliable)",
            "DWA for local planning (reactive)",
            "Hybrid A* + DWA (comprehensive)",
        ],
        "context": "Handle rough terrain and obstacles",
    },
}


def quick_decide(topic_key: str) -> Dict[str, Any]:
    """Make a quick decision from common options."""
    if topic_key not in COMMON_DECISIONS:
        print(f"❌ Unknown decision topic: {topic_key}")
        return None

    decision_config = COMMON_DECISIONS[topic_key]
    framework = DecisionFramework()

    return framework.make_decision(decision_config["topic"], decision_config["options"], decision_config["context"])


def main():
    """Interactive decision making."""
    print("🎯 AUTONOMY DECISION FRAMEWORK")
    print("=" * 40)

    framework = DecisionFramework()

    # Check for decisions needing review
    needs_review = framework.review_decisions()
    if needs_review:
        print(f"📅 DECISIONS NEEDING REVIEW ({len(needs_review)}):")
        for topic in needs_review:
            print(f"  • {topic}")

    print("\nAvailable quick decisions:")
    for key in COMMON_DECISIONS.keys():
        print(f"  • {key}")

    # Interactive mode
    while True:
        print("\nOptions:")
        print("  1. Make quick decision")
        print("  2. Review pending decisions")
        print("  3. View decision history")
        print("  4. Exit")

        choice = input("\nChoice: ").strip()

        if choice == "1":
            topic = input("Decision topic: ").strip()
            if topic in COMMON_DECISIONS:
                quick_decide(topic)
            else:
                print("❌ Topic not found. Available:", list(COMMON_DECISIONS.keys()))

        elif choice == "2":
            needs_review = framework.review_decisions()
            if needs_review:
                for topic in needs_review:
                    outcome = input(f"Outcome for '{topic}' (success/failure/modify): ").strip()
                    if outcome:
                        framework.close_decision(topic, outcome)
            else:
                print("✅ No decisions need review")

        elif choice == "3":
            print("\nDECISION HISTORY:")
            for topic, decision in framework.decisions.items():
                status = decision["status"]
                chosen = decision["chosen_option"]
                confidence = decision.get("confidence", "N/A")
                print(f"  {status.upper()}: {topic} → {chosen} ({confidence}%)")

        elif choice == "4":
            break


if __name__ == "__main__":
    main()
