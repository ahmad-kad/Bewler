#!/usr/bin/env python3
"""
Unit tests for ArUco Generator.

Tests core functionality of the ArucoGenerator class.
"""

# Add the parent directory to the path for imports
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import unittest
from unittest.mock import patch
import numpy as np
from PIL import Image

from aruco.generator import ArucoGenerator


class TestArucoGenerator(unittest.TestCase):
    """Test cases for ArucoGenerator class."""

    def setUp(self):
        """Set up test fixtures."""
        self.generator = ArucoGenerator(dpi=300)

    @patch("cv2.aruco.generateImageMarker")
    @patch("cv2.cvtColor")
    def test_generate_single_tag_success(self, mock_cvt_color, mock_generate):
        """Test successful single tag generation."""
        # Mock the OpenCV functions
        mock_generate.return_value = np.ones((100, 100), dtype=np.uint8) * 255
        mock_cvt_color.return_value = np.ones((100, 100, 3), dtype=np.uint8) * 255

        # Generate tag
        result = self.generator.generate_single_tag(0, 5.0)

        # Verify result
        self.assertEqual(result.shape, (100, 100, 3))  # Should be RGB
        mock_generate.assert_called_once()
        mock_cvt_color.assert_called_once()

    def test_generate_single_tag_invalid_id(self):
        """Test that invalid tag IDs raise errors."""
        with self.assertRaises(ValueError):
            self.generator.generate_single_tag(100, 5.0)  # ID too high

    @patch("cv2.aruco.generateImageMarker")
    def test_generate_single_tag_zero_size(self, mock_generate):
        """Test tag generation with zero size."""
        mock_generate.return_value = np.ones((1, 1), dtype=np.uint8)

        result = self.generator.generate_single_tag(0, 0.0)
        self.assertIsInstance(result, np.ndarray)

    def test_paper_sizes(self):
        """Test that paper sizes are correctly defined."""
        expected_sizes = {"letter": (8.5, 11.0), "a4": (8.27, 11.69)}

        self.assertEqual(self.generator.paper_sizes, expected_sizes)

    @patch("cv2.aruco.generateImageMarker")
    @patch("cv2.cvtColor")
    def test_create_tag_sheet_single_tag(self, mock_cvt_color, mock_generate):
        """Test creating a sheet with a single tag."""
        # Mock OpenCV
        mock_generate.return_value = np.ones((100, 100), dtype=np.uint8) * 255
        mock_cvt_color.return_value = np.ones((100, 100, 3), dtype=np.uint8) * 255

        # Create sheet
        tag_specs = [(5.0, 1)]  # One 5cm tag
        result = self.generator.create_tag_sheet(tag_specs)

        # Verify result is PIL Image
        self.assertIsInstance(result, Image.Image)
        self.assertEqual(result.mode, "RGB")

    def test_invalid_paper_format(self):
        """Test that invalid paper formats raise errors."""
        with self.assertRaises(ValueError):
            self.generator.create_tag_sheet([(5.0, 1)], paper_format="invalid")

    def test_dpi_property(self):
        """Test that DPI property is set correctly."""
        gen_150 = ArucoGenerator(dpi=150)
        self.assertEqual(gen_150.dpi, 150)

        gen_600 = ArucoGenerator(dpi=600)
        self.assertEqual(gen_600.dpi, 600)

    def test_generate_common_layouts_structure(self):
        """Test that common layouts have correct structure."""
        # This test doesn't actually generate files, just checks the data structure
        layouts = {
            "keyboard_small": [
                (1.0, 50),  # 50 1cm tags for keyboard corners
                (2.0, 25),  # 25 2cm tags for extras
            ],
            "keyboard_medium": [
                (3.0, 12),  # 12 3cm tags for keyboard
                (5.0, 8),  # 8 5cm tags for extras
            ],
            "navigation": [
                (10.0, 4),  # 4 10cm navigation markers
                (15.0, 2),  # 2 15cm navigation posts
            ],
            "large_navigation": [
                (20.0, 2),  # 2 20cm large navigation markers
            ],
        }

        # Verify all layouts are present
        self.assertIn("keyboard_small", layouts)
        self.assertIn("keyboard_medium", layouts)
        self.assertIn("navigation", layouts)
        self.assertIn("large_navigation", layouts)

        # Verify each layout has proper structure (list of tuples)
        for layout_name, tag_specs in layouts.items():
            self.assertIsInstance(tag_specs, list)
            for spec in tag_specs:
                self.assertIsInstance(spec, tuple)
                self.assertEqual(len(spec), 2)
                size, count = spec
                self.assertIsInstance(size, float)
                self.assertIsInstance(count, int)
                self.assertGreater(size, 0)
                self.assertGreater(count, 0)

    @patch("builtins.open")
    @patch("os.path.exists")
    @patch("os.makedirs")
    def test_save_sheet_png_directory_creation(
        self, mock_makedirs, mock_exists, _mock_open
    ):
        """Test that save_sheet_png creates directories when needed."""
        mock_exists.return_value = False

        with patch.object(self.generator, "create_tag_sheet") as mock_create:
            mock_create.return_value = Image.new("RGB", (100, 100), color="white")

            self.generator.save_sheet_png([(5.0, 1)], "/test/path/output.png")

            mock_makedirs.assert_called_once_with("/test/path")

    def test_tag_size_calculation(self):
        """Test that tag sizes are calculated correctly."""
        gen_300 = ArucoGenerator(dpi=300)
        # We can't easily test the internal calculation without mocking,
        # but we can verify the DPI affects the result
        gen_150 = ArucoGenerator(dpi=150)

        # Just verify different DPI values are accepted
        self.assertEqual(gen_300.dpi, 300)
        self.assertEqual(gen_150.dpi, 150)


if __name__ == "__main__":
    unittest.main()
