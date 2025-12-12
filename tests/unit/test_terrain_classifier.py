#!/usr/bin/env python3
"""
Unit tests for experimental TerrainClassifier.

These tests ensure the experimental API is at least internally consistent
and behaves as documented (placeholders, reserved methods, etc.).
"""

import os
import sys
from typing import Any, Dict

import pytest

# Add Autonomy/code to path so we can import experimental modules directly
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
AUTONOMY_CODE_ROOT = os.path.join(PROJECT_ROOT, "Autonomy", "code")
sys.path.insert(0, AUTONOMY_CODE_ROOT)

from experimental.terrain_classifier import (  # type: ignore  # noqa: E402
    TerrainClassifier,
    TerrainProperties,
    TerrainType,
)


class TestTerrainClassifierBasics:
    """Basic behavior of TerrainClassifier and enums."""

    def test_terrain_type_enum(self) -> None:
        """Enum members exist and have expected string values."""
        assert TerrainType.SAND.value == "sand"
        assert TerrainType.ROCK.value == "rock"
        assert TerrainType.GRAVEL.value == "gravel"
        assert TerrainType.SLOPE.value == "slope"
        assert TerrainType.OBSTACLE.value == "obstacle"
        assert TerrainType.UNKNOWN.value == "unknown"

    def test_properties_structure(self) -> None:
        """TerrainProperties dataclass has required fields."""
        props = TerrainProperties(
            type=TerrainType.SAND,
            traversability=0.5,
            max_speed=1.0,
            traction_coefficient=0.8,
            roughness=0.2,
            risk_level="medium",
        )
        assert props.type == TerrainType.SAND
        assert 0.0 <= props.traversability <= 1.0
        assert props.max_speed > 0.0
        assert isinstance(props.risk_level, str)


class TestTerrainClassifierAPI:
    """Test the experimental API surface of TerrainClassifier."""

    @pytest.fixture
    def classifier(self) -> TerrainClassifier:
        return TerrainClassifier()

    def test_default_properties_exist(self, classifier: TerrainClassifier) -> None:
        """Default terrain properties are populated for known types."""
        for terrain_type in [
            TerrainType.SAND,
            TerrainType.ROCK,
            TerrainType.GRAVEL,
            TerrainType.SLOPE,
        ]:
            props = classifier.get_terrain_properties(terrain_type)
            assert isinstance(props, TerrainProperties)
            assert props.type == terrain_type

    def test_unknown_properties_fallback(self, classifier: TerrainClassifier) -> None:
        """Unknown terrain types fall back to UNKNOWN properties."""
        props_unknown = classifier.get_terrain_properties(TerrainType.UNKNOWN)
        assert isinstance(props_unknown, TerrainProperties)
        assert props_unknown.type == TerrainType.UNKNOWN

    def test_classify_terrain_placeholder(self, classifier: TerrainClassifier) -> None:
        """classify_terrain currently returns UNKNOWN as documented."""
        dummy_sensor_data: Dict[str, Any] = {"dummy": 1}
        terrain_type = classifier.classify_terrain(dummy_sensor_data)
        assert terrain_type == TerrainType.UNKNOWN

    def test_assess_traversability_placeholder(self, classifier: TerrainClassifier) -> None:
        """assess_traversability returns a plausible placeholder score."""
        score = classifier.assess_traversability((0.0, 0.0))
        assert isinstance(score, float)
        assert 0.0 <= score <= 1.0

    def test_recommend_speed_respects_max(self, classifier: TerrainClassifier) -> None:
        """recommend_speed should not exceed max_speed for terrain."""
        current_speed = 2.0  # m/s
        props = classifier.get_terrain_properties(TerrainType.SAND)
        recommended = classifier.recommend_speed(TerrainType.SAND, current_speed)
        assert recommended <= props.max_speed

    def test_calculate_terrain_cost_positive(self, classifier: TerrainClassifier) -> None:
        """calculate_terrain_cost returns positive cost proportional to distance."""
        distance = 10.0
        cost = classifier.calculate_terrain_cost(TerrainType.GRAVEL, distance)
        assert cost > 0.0

        # Doubling distance should increase cost (monotonic behavior)
        cost2 = classifier.calculate_terrain_cost(TerrainType.GRAVEL, distance * 2)
        assert cost2 > cost

    def test_detect_hazards_placeholder(self, classifier: TerrainClassifier) -> None:
        """detect_hazards currently returns an empty list as placeholder."""
        hazards = classifier.detect_hazards({"dummy": True})
        assert isinstance(hazards, list)
        assert len(hazards) == 0

    def test_update_and_reset_dont_crash(self, classifier: TerrainClassifier) -> None:
        """update_terrain_map and reset_classifier are no-ops but must not crash."""
        classifier.update_terrain_map((0.0, 0.0), TerrainType.SAND)
        classifier.reset_classifier()
        classifier.shutdown()
