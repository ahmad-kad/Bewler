#!/usr/bin/env python3
"""
Simplified ArUco Tag Generator - URC 2026

Core functionality: Generate ArUco tags in simple layouts for navigation and typing.
Reduced complexity while maintaining essential features.

Features:
- Single tag generation
- Simple grid layouts for sheets
- PNG output at paper size
- Basic tag sizing for common use cases
"""

import argparse
import os
from typing import List, Tuple

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


class ArucoGenerator:
    """Simplified ArUco tag generator with essential features only."""

    def __init__(self, dpi: int = 300):
        self.dpi = dpi
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.max_id = self.aruco_dict.bytesList.shape[0] - 1

        # Standard paper sizes
        self.paper_sizes = {
            "letter": (8.5, 11.0),  # US Letter
            "a4": (8.27, 11.69),  # A4
        }

    def generate_single_tag(self, tag_id: int, size_cm: float) -> np.ndarray:
        """Generate a single ArUco tag.

        Args:
            tag_id: ArUco marker ID (0-49)
            size_cm: Tag size in centimeters

        Returns:
            RGB numpy array of the tag
        """
        if tag_id > self.max_id:
            raise ValueError(f"Tag ID {tag_id} exceeds maximum {self.max_id}")

        # Convert cm to pixels
        size_px = int(size_cm / 2.54 * self.dpi)  # cm -> inches -> pixels

        # Generate tag
        gray_tag = cv2.aruco.generateImageMarker(self.aruco_dict, tag_id, size_px)
        rgb_tag = cv2.cvtColor(gray_tag, cv2.COLOR_GRAY2RGB)

        return rgb_tag

    def create_tag_sheet(
        self,
        tag_specs: List[Tuple[float, int]],
        paper_format: str = "letter",
        start_id: int = 0,
    ) -> Image.Image:
        """Create a sheet of ArUco tags in a simple grid layout.

        Args:
            tag_specs: List of (size_cm, count) tuples
            paper_format: Paper size ("letter" or "a4")
            start_id: Starting tag ID

        Returns:
            PIL Image of the tag sheet at paper size
        """
        if paper_format not in self.paper_sizes:
            raise ValueError(f"Unknown paper format: {paper_format}")

        paper_width_inches, paper_height_inches = self.paper_sizes[paper_format]

        # Convert to pixels
        paper_width_px = int(paper_width_inches * self.dpi)
        paper_height_px = int(paper_height_inches * self.dpi)

        # Create white canvas
        canvas = np.ones((paper_height_px, paper_width_px, 3), dtype=np.uint8) * 255

        # Simple layout parameters
        margin_inches = 0.5  # 0.5" margin on all sides
        margin_px = int(margin_inches * self.dpi)

        usable_width_px = paper_width_px - 2 * margin_px

        current_y = margin_px
        current_id = start_id

        # Process each tag specification
        for size_cm, count in tag_specs:
            if current_id > self.max_id:
                print(f"Warning: Ran out of tag IDs at {current_id}")
                break

            # Generate tag
            tag_img = self.generate_single_tag(current_id, size_cm)
            tag_height_px, tag_width_px = tag_img.shape[:2]

            # Simple horizontal layout (left to right, then down)
            tags_per_row = max(1, usable_width_px // tag_width_px)
            row_spacing_px = int(0.25 * self.dpi)  # 0.25" spacing

            for i in range(count):
                if current_id > self.max_id:
                    break

                # Calculate position
                row = i // tags_per_row
                col = i % tags_per_row

                x = margin_px + col * tag_width_px
                y = current_y + row * (tag_height_px + row_spacing_px)

                # Check if it fits
                if y + tag_height_px > paper_height_px - margin_px:
                    print(f"Warning: Not enough space for remaining {count - i} tags")
                    break

                # Place tag on canvas
                canvas[y : y + tag_height_px, x : x + tag_width_px] = tag_img

                # Add simple label
                self._add_tag_label(canvas, current_id, x, y + tag_height_px + 5)

                current_id += 1

            # Move to next section
            rows_used = (count + tags_per_row - 1) // tags_per_row
            current_y += rows_used * (tag_height_px + row_spacing_px) + int(
                0.5 * self.dpi
            )

        return Image.fromarray(canvas)

    def _add_tag_label(self, canvas: np.ndarray, tag_id: int, x: int, y: int):
        """Add a simple ID label below a tag."""
        pil_img = Image.fromarray(canvas)
        draw = ImageDraw.Draw(pil_img)

        try:
            # Try to load a simple font
            font = ImageFont.load_default()
        except (IOError, OSError):
            font = ImageFont.load_default()

        label = f"ID:{tag_id}"
        draw.text((x, y), label, fill=(0, 0, 0), font=font)

        # Convert back to numpy array
        canvas[:] = np.array(pil_img)

    def generate_single_tag_pdf(self, tag_id: int, size_cm: float, output_path: str):
        """Generate a single ArUco tag and save as PDF (like generate_aruco_tag.py).

        This preserves the PDF export functionality from the legacy generator.
        """
        if tag_id > self.max_id:
            raise ValueError(f"Tag ID {tag_id} exceeds maximum {self.max_id}")

        # Generate the tag
        tag_array = self.generate_single_tag(tag_id, size_cm)

        # Convert to PIL Image
        tag_img = Image.fromarray(tag_array)

        # Create a larger canvas with border and label (like the original)
        border_px = 50
        label_height_px = 30

        canvas_width = tag_img.width + 2 * border_px
        canvas_height = tag_img.height + 2 * border_px + label_height_px

        canvas = Image.new("RGB", (canvas_width, canvas_height), "white")
        canvas.paste(tag_img, (border_px, border_px))

        # Add ID label below the tag
        draw = ImageDraw.Draw(canvas)
        try:
            font = ImageFont.load_default()
        except (IOError, OSError):
            font = ImageFont.load_default()

        label_text = f"ID:{tag_id} ({size_cm}cm)"
        bbox = draw.textbbox((0, 0), label_text, font=font)
        text_width = bbox[2] - bbox[0]
        text_x = (canvas_width - text_width) // 2
        text_y = border_px + tag_img.height + border_px // 2

        draw.text((text_x, text_y), label_text, fill=(0, 0, 0), font=font)

        # Save as PDF
        canvas.save(output_path, "PDF", resolution=self.dpi)
        print(f"Saved ArUco tag ID {tag_id} ({size_cm}cm) to {output_path}")

    def save_sheet_png(
        self,
        tag_specs: List[Tuple[float, int]],
        output_path: str,
        paper_format: str = "letter",
        start_id: int = 0,
    ):
        """Generate and save a tag sheet as PNG.

        Args:
            tag_specs: List of (size_cm, count) tuples
            output_path: Output PNG file path
            paper_format: Paper size ("letter" or "a4")
            start_id: Starting tag ID
        """
        # Create the sheet
        sheet_img = self.create_tag_sheet(tag_specs, paper_format, start_id)

        # Ensure output directory exists
        output_dir = os.path.dirname(output_path)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir)

        # Save with DPI metadata
        sheet_img.save(output_path, dpi=(self.dpi, self.dpi))

        # Get dimensions for reporting
        width_px, height_px = sheet_img.size
        width_inches = width_px / self.dpi
        height_inches = height_px / self.dpi

        print(f"Saved: {output_path}")
        print(
            f'  Size: {width_inches:.1f}" × {height_inches:.1f}" ({width_px}×{height_px}px)'
        )
        print(f"  DPI: {self.dpi}")

    def generate_common_layouts(self, output_dir: str = "aruco_tags"):
        """Generate common tag layouts for URC 2026 use cases."""
        layouts = {
            "keyboard_small": [
                (1.0, 50),  # 50× 1cm tags for keyboard corners
                (2.0, 25),  # 25× 2cm tags for extras
            ],
            "keyboard_medium": [
                (3.0, 12),  # 12× 3cm tags for keyboard
                (5.0, 8),  # 8× 5cm tags for extras
            ],
            "navigation": [
                (10.0, 4),  # 4× 10cm navigation markers
                (15.0, 2),  # 2× 15cm navigation posts
            ],
            "large_navigation": [
                (20.0, 2),  # 2× 20cm large navigation markers
            ],
        }

        os.makedirs(output_dir, exist_ok=True)

        print(f"Generating common layouts in: {output_dir}")
        print("=" * 50)

        for layout_name, tag_specs in layouts.items():
            output_path = os.path.join(output_dir, f"{layout_name}_tags.png")

            print(f"\nGenerating {layout_name} layout...")
            self.save_sheet_png(tag_specs, output_path)

        print("\nAll layouts generated!")
        print(f"  Location: {os.path.abspath(output_dir)}")


def main():
    """Command line interface for the simplified ArUco generator."""
    parser = argparse.ArgumentParser(
        description="Simple ArUco Tag Generator - URC 2026",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python aruco_generator.py --layout keyboard_small
  python aruco_generator.py --single 0 5.0 --output tag_0_5cm.png
  python aruco_generator.py --pdf-tag 42 10 --output navigation_marker_42.pdf
  python aruco_generator.py --custom "2:10,5:4" --output custom_tags.png
        """,
    )

    parser.add_argument(
        "--layout",
        choices=["keyboard_small", "keyboard_medium", "navigation", "large_navigation"],
        help="Generate a predefined layout",
    )

    parser.add_argument(
        "--single",
        nargs=2,
        metavar=("ID", "SIZE_CM"),
        help="Generate a single tag as PNG sheet (ID and size in cm)",
    )
    parser.add_argument(
        "--pdf-tag",
        nargs=2,
        metavar=("ID", "SIZE_CM"),
        help="Generate a single tag as PDF (like generate_aruco_tag.py)",
    )

    parser.add_argument(
        "--custom", help='Custom layout: "size1:count1,size2:count2" (e.g., "2:10,5:4")'
    )

    parser.add_argument("--output", "-o", default=None, help="Output file path")

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

    generator = ArucoGenerator(dpi=args.dpi)

    if args.layout:
        # Generate predefined layout
        output_dir = args.output or f"{args.layout}_tags"
        generator.generate_common_layouts(output_dir)

    elif args.pdf_tag:
        # Generate single tag as PDF (legacy compatibility)
        tag_id = int(args.pdf_tag[0])
        size_cm = float(args.pdf_tag[1])
        output_path = args.output or f"aruco_id_{tag_id}_{size_cm}cm.pdf"

        print(f"Generating PDF tag: ID {tag_id}, {size_cm}cm")
        generator.generate_single_tag_pdf(tag_id, size_cm, output_path)

    elif args.single:
        # Generate single tag as PNG sheet
        tag_id = int(args.single[0])
        size_cm = float(args.single[1])
        output_path = args.output or f"aruco_id_{tag_id}_{size_cm}cm.png"

        print(f"Generating single tag sheet: ID {tag_id}, {size_cm}cm")
        generator.save_sheet_png([(size_cm, 1)], output_path, args.paper, tag_id)

    elif args.custom:
        # Parse custom layout
        try:
            tag_specs = []
            for spec in args.custom.split(","):
                size_str, count_str = spec.split(":")
                size_cm = float(size_str.strip())
                count = int(count_str.strip())
                tag_specs.append((size_cm, count))

            output_path = args.output or "custom_aruco_tags.png"
            print(f"Generating custom layout with {len(tag_specs)} tag types")
            generator.save_sheet_png(tag_specs, output_path, args.paper)

        except ValueError as e:
            print(f"Error parsing custom layout: {e}")
            print("Format: --custom 'size1:count1,size2:count2'")
            return 1

    else:
        # Default: show available layouts
        print("Available layouts:")
        print("  keyboard_small   - Small tags for keyboard (1cm, 2cm)")
        print("  keyboard_medium  - Medium tags for keyboard (3cm, 5cm)")
        print("  navigation       - Navigation markers (10cm, 15cm)")
        print("  large_navigation - Large navigation markers (20cm)")
        print("\nUse --layout <name> to generate a layout")
        return 0

    return 0


if __name__ == "__main__":
    exit(main())
