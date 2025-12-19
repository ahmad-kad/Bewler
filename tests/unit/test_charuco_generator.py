#!/usr/bin/env python3
"""
Unit tests for ChArUco Generator.

Tests core functionality of the CharucoGenerator class.
"""

import unittest
from unittest.mock import patch, MagicMock
import numpy as np

# Add the parent directory to the path for imports
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from charuco.generator import CharucoGenerator


class TestCharucoGenerator(unittest.TestCase):
    """Test cases for CharucoGenerator class."""

    def setUp(self):
        """Set up test fixtures."""
        self.generator = CharucoGenerator(dpi=300)

    def test_initialization(self):
        """Test that generator initializes correctly."""
        self.assertEqual(self.generator.dpi, 300)
        self.assertIn("small", self.generator.standard_sizes)
        self.assertIn("medium", self.generator.standard_sizes)
        self.assertIn("large", self.generator.standard_sizes)

    def test_standard_sizes_structure(self):
        """Test that standard sizes have correct structure."""
        for size_name, config in self.generator.standard_sizes.items():
            self.assertIn("rows", config)
            self.assertIn("cols", config)
            self.assertIn("checker_mm", config)
            self.assertIn("marker_mm", config)

            # Verify values are reasonable
            self.assertGreater(config["rows"], 0)
            self.assertGreater(config["cols"], 0)
            self.assertGreater(config["checker_mm"], 0)
            self.assertGreater(config["marker_mm"], 0)
            self.assertLess(config["marker_mm"], config["checker_mm"])

    @patch("cv2.aruco.CharucoBoard")
    def test_generate_board_basic(self, mock_board_class):
        """Test basic board generation."""
        # Mock the board and its methods
        mock_board = MagicMock()
        mock_board.generateImage.return_value = np.ones((200, 300), dtype=np.uint8)
        mock_board_class.return_value = mock_board

        result = self.generator.generate_board(
            rows=5, cols=7, checker_size_mm=30, marker_size_mm=18
        )

        # Verify result is numpy array with correct shape (paper size)
        self.assertIsInstance(result, np.ndarray)
        self.assertEqual(result.shape[2], 3)  # RGB

        # Verify board generation was called
        mock_board_class.assert_called_once_with(
            (7, 5), 30, 18, self.generator.aruco_dict
        )
        mock_board.generateImage.assert_called_once()

    def test_invalid_paper_format(self):
        """Test that invalid paper formats raise errors."""
        with self.assertRaises(ValueError):
            self.generator.generate_board(paper_format="invalid")

    def test_paper_size_calculations(self):
        """Test that paper sizes are calculated correctly."""
        # Letter: 8.5" x 11" at 300 DPI = 2550 x 3300 pixels
        gen_300 = CharucoGenerator(dpi=300)

        # Test letter size
        result = gen_300.generate_board(rows=5, cols=7, paper_format="letter")
        expected_width = int(8.5 * 300)  # 2550
        expected_height = int(11.0 * 300)  # 3300
        self.assertEqual(result.shape[1], expected_width)  # width
        self.assertEqual(result.shape[0], expected_height)  # height

    def test_different_dpi_values(self):
        """Test that different DPI values affect output size."""
        gen_150 = CharucoGenerator(dpi=150)
        gen_300 = CharucoGenerator(dpi=300)

        # Same board, different DPI should give different pixel dimensions
        result_150 = gen_150.generate_board(rows=5, cols=7)
        result_300 = gen_300.generate_board(rows=5, cols=7)

        # 300 DPI should be exactly double 150 DPI dimensions
        self.assertEqual(result_300.shape[0], result_150.shape[0] * 2)  # height
        self.assertEqual(result_300.shape[1], result_150.shape[1] * 2)  # width

    @patch("cv2.aruco.CharucoBoard")
    def test_board_placement_centering(self, mock_board_class):
        """Test that boards are centered on the paper."""
        mock_board = MagicMock()
        mock_board.generateImage.return_value = (
            np.ones((100, 100), dtype=np.uint8) * 128
        )
        mock_board_class.return_value = mock_board

        result = self.generator.generate_board(rows=5, cols=7)

        # Board should be centered, so edges should be white (background)
        height, width = result.shape[:2]

        # Top-left corner should be white (background)
        self.assertTrue(np.allclose(result[0, 0], [255, 255, 255], atol=10))

        # Some area in the middle should have the board content (gray)
        center_y, center_x = height // 2, width // 2
        self.assertFalse(
            np.allclose(result[center_y, center_x], [255, 255, 255], atol=10)
        )

    @patch("os.makedirs")
    @patch("os.path.exists")
    @patch("builtins.open")
    def test_save_board_png_directory_creation(
        self, _mock_open, mock_exists, mock_makedirs
    ):
        """Test that save_board_png creates directories when needed."""
        mock_exists.return_value = False

        with patch("PIL.Image.fromarray") as mock_fromarray:
            mock_image = MagicMock()
            mock_fromarray.return_value = mock_image

            self.generator.save_board_png(
                rows=5, cols=7, output_path="/test/path/board.png"
            )

            mock_makedirs.assert_called_once_with("/test/path")

    def test_generate_standard_boards_keys(self):
        """Test that generate_standard_boards processes all expected sizes."""
        expected_sizes = ["small", "medium", "large"]

        with patch.object(self.generator, "save_board_png") as mock_save:
            self.generator.generate_standard_boards(output_dir="/tmp")

            # Should call save_board_png for each size
            self.assertEqual(mock_save.call_count, len(expected_sizes))

            # Verify each size was processed
            called_sizes = set()
            for call in mock_save.call_args_list:
                kwargs = call[1]
                for size_name, config in self.generator.standard_sizes.items():
                    if (
                        kwargs["rows"] == config["rows"]
                        and kwargs["cols"] == config["cols"]
                    ):
                        called_sizes.add(size_name)
                        break

            self.assertEqual(called_sizes, set(expected_sizes))

    def test_board_size_validation(self):
        """Test that board parameters are validated."""
        # Valid parameters should work
        try:
            self.generator.generate_board(
                rows=5, cols=7, checker_size_mm=30, marker_size_mm=18
            )
        except Exception:
            self.fail("Valid board parameters should not raise exceptions")

    def test_a4_paper_format(self):
        """Test A4 paper format."""
        result = self.generator.generate_board(rows=5, cols=7, paper_format="a4")

        # A4 at 300 DPI: 8.27" x 11.69" = 2481 x 3507 pixels (approximately)
        expected_width = int(8.27 * 300)
        expected_height = int(11.69 * 300)

        self.assertEqual(result.shape[1], expected_width)
        self.assertEqual(result.shape[0], expected_height)


if __name__ == "__main__":
    unittest.main()
