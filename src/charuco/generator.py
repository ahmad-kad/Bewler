#!/usr/bin/env python3
"""
Simplified ChArUco Board Generator - URC 2026

Core functionality: Generate ChArUco boards for camera calibration.
Reduced complexity while maintaining calibration accuracy.

Features:
- Basic ChArUco board generation
- Standard calibration patterns
- PNG output for easy use
"""

import argparse
import os

import cv2
import numpy as np


class CharucoGenerator:
    """Simplified ChArUco board generator for camera calibration."""

    def __init__(self, dpi: int = 300):
        self.dpi = dpi
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

        # Standard calibration board sizes
        self.standard_sizes = {
            "small": {"rows": 5, "cols": 7, "checker_mm": 20, "marker_mm": 12},
            "medium": {"rows": 7, "cols": 5, "checker_mm": 30, "marker_mm": 18},
            "large": {"rows": 9, "cols": 7, "checker_mm": 40, "marker_mm": 24},
        }

    def generate_board(
        self,
        rows: int = 7,
        cols: int = 5,
        checker_size_mm: float = 30.0,
        marker_size_mm: float = 18.0,
        paper_format: str = "letter",
    ) -> np.ndarray:
        """Generate a ChArUco board image.

        Args:
            rows: Number of square rows
            cols: Number of square columns
            checker_size_mm: Size of checker squares in mm
            marker_size_mm: Size of ArUco markers in mm
            paper_format: Paper size ("letter" or "a4")

        Returns:
            RGB numpy array of the board at paper size
        """
        # Create ChArUco board
        board = cv2.aruco.CharucoBoard(
            (cols, rows), checker_size_mm, marker_size_mm, self.aruco_dict
        )

        # Calculate board dimensions in pixels
        board_width_mm = cols * checker_size_mm
        board_height_mm = rows * checker_size_mm

        # Convert mm to inches (1 inch = 25.4 mm)
        board_width_inches = board_width_mm / 25.4
        board_height_inches = board_height_mm / 25.4

        # Paper dimensions
        paper_sizes = {"letter": (8.5, 11.0), "a4": (8.27, 11.69)}

        if paper_format not in paper_sizes:
            raise ValueError(f"Unknown paper format: {paper_format}")

        paper_width_inches, paper_height_inches = paper_sizes[paper_format]

        # Convert to pixels
        paper_width_px = int(paper_width_inches * self.dpi)
        paper_height_px = int(paper_height_inches * self.dpi)

        # Calculate board size in pixels
        board_width_px = int(board_width_inches * self.dpi)
        board_height_px = int(board_height_inches * self.dpi)

        # Generate board image
        board_img = board.generateImage((board_width_px, board_height_px))

        # Convert to RGB if needed
        if len(board_img.shape) == 2:
            board_img = cv2.cvtColor(board_img, cv2.COLOR_GRAY2RGB)

        # Create white canvas at paper size
        canvas = np.ones((paper_height_px, paper_width_px, 3), dtype=np.uint8) * 255

        # Center the board on the page
        start_x = (paper_width_px - board_width_px) // 2
        start_y = (paper_height_px - board_height_px) // 2

        # Place board on canvas
        if start_x >= 0 and start_y >= 0:
            canvas[
                start_y : start_y + board_height_px, start_x : start_x + board_width_px
            ] = board_img

        return canvas

    def save_board_png(
        self,
        rows: int = 7,
        cols: int = 5,
        checker_size_mm: float = 30.0,
        marker_size_mm: float = 18.0,
        output_path: str = "charuco_board.png",
        paper_format: str = "letter",
    ):
        """Generate and save a ChArUco board as PNG.

        Args:
            rows: Number of square rows
            cols: Number of square columns
            checker_size_mm: Size of checker squares in mm
            marker_size_mm: Size of ArUco markers in mm
            output_path: Output PNG file path
            paper_format: Paper size ("letter" or "a4")
        """
        # Generate board
        board_img = self.generate_board(
            rows, cols, checker_size_mm, marker_size_mm, paper_format
        )

        # Ensure output directory exists
        output_dir = os.path.dirname(output_path)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir)

        # Convert to PIL and save
        from PIL import Image

        pil_img = Image.fromarray(board_img)
        pil_img.save(output_path, dpi=(self.dpi, self.dpi))

        # Calculate physical dimensions
        height_px, width_px = board_img.shape[:2]
        width_inches = width_px / self.dpi
        height_inches = height_px / self.dpi

        print(f"Saved ChArUco board: {output_path}")
        print(f"  Board size: {cols}x{rows} squares")
        print(f"  Checker size: {checker_size_mm}mm")
        print(f"  Marker size: {marker_size_mm}mm")
        print(f'  Paper size: {width_inches:.1f}" × {height_inches:.1f}"')
        print(f"  Pixel size: {width_px}×{height_px}")

    def generate_standard_boards(self, output_dir: str = "charuco_boards"):
        """Generate standard calibration boards for common use cases."""
        os.makedirs(output_dir, exist_ok=True)

        print(f"Generating standard ChArUco boards in: {output_dir}")
        print("=" * 50)

        for size_name, config in self.standard_sizes.items():
            output_path = os.path.join(output_dir, f"charuco_{size_name}.png")

            print(f"\nGenerating {size_name} calibration board...")
            self.save_board_png(
                rows=config["rows"],
                cols=config["cols"],
                checker_size_mm=config["checker_mm"],
                marker_size_mm=config["marker_mm"],
                output_path=output_path,
            )

        print("\nAll standard boards generated!")
        print(f"  Location: {os.path.abspath(output_dir)}")


def main():
    """Command line interface for the simplified ChArUco generator."""
    parser = argparse.ArgumentParser(
        description="Simple ChArUco Board Generator - URC 2026",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python simple_charuco_generator.py --standard
  python simple_charuco_generator.py --custom 7 5 30 18 --output custom_board.png
  python simple_charuco_generator.py --size medium --output medium_board.png
        """,
    )

    parser.add_argument(
        "--standard",
        action="store_true",
        help="Generate all standard calibration boards",
    )

    parser.add_argument(
        "--size",
        choices=["small", "medium", "large"],
        help="Generate a standard size board",
    )

    parser.add_argument(
        "--custom",
        nargs=4,
        metavar=("ROWS", "COLS", "CHECKER_MM", "MARKER_MM"),
        help="Generate custom board (rows, cols, checker_size_mm, marker_size_mm)",
    )

    parser.add_argument("--output", "-o", default=None, help="Output PNG file path")

    parser.add_argument(
        "--paper",
        choices=["letter", "a4"],
        default="letter",
        help="Paper format (default: letter)",
    )

    parser.add_argument(
        "--dpi", type=int, default=300, help="DPI for output (default: 300)"
    )

    args = parser.parse_args()

    generator = CharucoGenerator(dpi=args.dpi)

    if args.standard:
        # Generate all standard boards
        output_dir = args.output or "charuco_boards"
        generator.generate_standard_boards(output_dir)

    elif args.size:
        # Generate specific standard size
        config = generator.standard_sizes[args.size]
        output_path = args.output or f"charuco_{args.size}.png"

        print(f"Generating {args.size} ChArUco board...")
        generator.save_board_png(
            rows=config["rows"],
            cols=config["cols"],
            checker_size_mm=config["checker_mm"],
            marker_size_mm=config["marker_mm"],
            output_path=output_path,
            paper_format=args.paper,
        )

    elif args.custom:
        # Generate custom board
        rows = int(args.custom[0])
        cols = int(args.custom[1])
        checker_mm = float(args.custom[2])
        marker_mm = float(args.custom[3])

        output_path = (
            args.output or f"output/charuco_{rows}x{cols}_{checker_mm}_{marker_mm}.png"
        )

        print("Generating custom ChArUco board...")
        generator.save_board_png(
            rows=rows,
            cols=cols,
            checker_size_mm=checker_mm,
            marker_size_mm=marker_mm,
            output_path=output_path,
            paper_format=args.paper,
        )

    else:
        # Default: show available options
        print("Standard board sizes:")
        for name, config in generator.standard_sizes.items():
            print(
                f"  {name}: {config['cols']}x{config['rows']} squares, "
                f"{config['checker_mm']}mm checkers, {config['marker_mm']}mm markers"
            )

        print("\nUse --standard to generate all sizes")
        print("Use --size <name> for a specific standard size")
        print("Use --custom for custom dimensions")
        return 0

    return 0


if __name__ == "__main__":
    exit(main())
