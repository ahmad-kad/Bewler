#!/usr/bin/env python3
"""
ArUco Tag Generator for Environment Setup - URC 2026

Generates individual ArUco tags for environment setup and navigation markers.
These tags are used for:
- Navigation landmarks and waypoints
- Equipment/service area markers
- Autonomous typing keyboard and USB detection
- General environmental fiducials

NOTE: These ArUco tags are NOT used for camera calibration.
Use ChArUco boards (charuco_board/generate_chessboard.py) for camera intrinsics and hand-eye calibration.

Usage:
    python generate_aruco_tag.py --id 42 --size 10 --output navigation_marker_42.pdf
"""

import argparse

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


def generate_aruco_tag(marker_id=0, size_cm=20.0, aruco_dict_name="DICT_4X4_50",
                       output_filename="aruco_tag.pdf"):
    """
    Generates a single ArUco tag and saves it as a to-scale PDF.
    """
    print("--- ArUco Tag Configuration ---")
    print(f"OpenCV version: {cv2.__version__}")
    print(f"Marker ID: {marker_id}")
    print(f"Physical Size: {size_cm} x {size_cm} cm")
    print(f"ArUco Dictionary: {aruco_dict_name}")
    print("-------------------------------")

    try:
        aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, aruco_dict_name))
    except AttributeError:
        print(f"❌ Error: Dictionary '{aruco_dict_name}' not found.")
        return

    num_markers = aruco_dict.bytesList.shape[0]
    if not 0 <= marker_id < num_markers:
        print(f"❌ Error: Invalid marker ID '{marker_id}' for dictionary '{aruco_dict_name}'.")
        print(f"   Valid IDs are 0 to {num_markers - 1}.")
        return

    # --- Page and Tag Size Calculations ---
    US_LETTER_INCHES = (8.5, 11)
    DPI = 300
    CM_PER_INCH = 2.54

    tag_size_inches = size_cm / CM_PER_INCH
    tag_size_px = int(tag_size_inches * DPI)

    print(f"Tag physical size: {tag_size_inches:.2f} x {tag_size_inches:.2f} inches")

    # --- Image Generation ---
    img_gray = cv2.aruco.generateImageMarker(aruco_dict, marker_id, tag_size_px)
    img_rgb = cv2.cvtColor(img_gray, cv2.COLOR_GRAY2RGB)

    # Create a full page 3-channel (RGB) white canvas
    page_width_px = int(US_LETTER_INCHES[0] * DPI)
    page_height_px = int(US_LETTER_INCHES[1] * DPI)
    canvas = np.ones((page_height_px, page_width_px, 3), dtype=np.uint8) * 255

    # Calculate position to center the tag
    start_x = (page_width_px - tag_size_px) // 2
    start_y = (page_height_px - tag_size_px) // 2

    # Place the RGB tag on the RGB canvas
    canvas[start_y : start_y + tag_size_px, start_x : start_x + tag_size_px] = img_rgb

    # --- PDF Creation (using PIL) ---
    # Convert numpy array to PIL Image
    pil_image = Image.fromarray(canvas, mode='RGB')

    # Add footer text using ImageDraw
    draw = ImageDraw.Draw(pil_image)
    footer_text = (
        f"ArUco Tag | ID: {marker_id} | Dict: {aruco_dict_name} | "
        f"Size: {size_cm:.1f}x{size_cm:.1f} cm | Print @ 100% Scale"
    )

    # Position footer near bottom (20 pixels from bottom)
    footer_y = page_height_px - 30

    # Try to use a monospace font; fallback to default if not available
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 16)
    except (IOError, OSError):
        # Fallback to default font if system font not found
        font = ImageFont.load_default()

    draw.text((10, footer_y), footer_text, fill=(0, 0, 0), font=font)

    try:
        pil_image.save(output_filename, format='PDF')
        print(f"\n✅ Successfully generated tag: '{output_filename}'")
    except Exception as e:
        print(f"\n❌ Error saving file: {e}")


def main():
    parser = argparse.ArgumentParser(
        description='Generate a single, to-scale ArUco tag for printing.'
    )
    parser.add_argument(
        '-i', '--id', type=int, required=True,
        help='The ID of the ArUco marker to generate.'
    )
    parser.add_argument(
        '-s', '--size-cm', type=float, required=True,
        help='The physical size (width and height) of the tag in centimeters.'
    )
    parser.add_argument(
        '-d', '--dict', default='DICT_4X4_50',
        help='ArUco dictionary name (default: DICT_4X4_50).'
    )
    parser.add_argument(
        '-o', '--output', default=None,
        help='Output PDF filename. If not set, a name is generated automatically.'
    )
    args = parser.parse_args()

    output_filename = args.output
    if output_filename is None:
        output_filename = f"aruco_{args.dict}_id{args.id}_{args.size_cm:.0f}cm.pdf"

    generate_aruco_tag(
        marker_id=args.id,
        size_cm=args.size_cm,
        aruco_dict_name=args.dict,
        output_filename=output_filename
    )


if __name__ == '__main__':
    main()
