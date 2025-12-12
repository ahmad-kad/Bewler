#!/usr/bin/env python3
"""
ChArUco Board Generator for Camera Calibration

Generates a ChArUco board and saves it as a PDF formatted for a US Letter page.
"""

import argparse

import cv2
import matplotlib.pyplot as plt
import numpy as np


def generate_charuco_board(rows=7, cols=5, checker_size_mm=30.0, marker_size_mm=18.0,
                          aruco_dict_name="DICT_4X4_50", output_filename="charuco_board.pdf"):
    """
    Generates a ChArUco board and saves it as a PDF formatted for a US Letter page.
    """

    print("--- Configuration ---")
    print(f"Board Dimensions: {cols}x{rows}")
    print(f"Checker Size: {checker_size_mm} mm")
    print(f"Marker Size: {marker_size_mm} mm")
    print(f"ArUco Dictionary: {aruco_dict_name}")
    print("---------------------")

    # Get the predefined ArUco dictionary
    try:
        aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, aruco_dict_name))
    except AttributeError:
        print(f"❌ Error: Dictionary '{aruco_dict_name}' not found in cv2.aruco.")
        return

    # Create the ChArUco board object using the modern constructor
    board = cv2.aruco.CharucoBoard((cols, rows), checker_size_mm, marker_size_mm, aruco_dict)

    # --- Page and Board Size Calculations ---
    CANVAS_SIZE_PX = (3301, 2550)  # Custom canvas size in pixels
    DPI = 300 # Use high DPI for print quality
    MM_PER_INCH = 25.4

    # Calculate board physical size in mm and inches
    board_width_mm = cols * checker_size_mm
    board_height_mm = rows * checker_size_mm
    board_width_inches = board_width_mm / MM_PER_INCH
    board_height_inches = board_height_mm / MM_PER_INCH

    print(f"Board physical size: {board_width_mm:.1f} x {board_height_mm:.1f} mm")
    print(f"Board physical size: {board_width_inches:.2f} x {board_height_inches:.2f} inches")

    # Warn if the board is too large for the canvas
    canvas_width_inches = CANVAS_SIZE_PX[0] / DPI
    canvas_height_inches = CANVAS_SIZE_PX[1] / DPI
    if board_width_inches > canvas_width_inches - 1.0 or board_height_inches > canvas_height_inches - 1.0:
        print(f"\n⚠️ WARNING: Board dimensions may be too large for the {canvas_width_inches:.1f}\"x{canvas_height_inches:.1f}\" canvas with margins.")

    # Calculate board dimensions in pixels
    board_width_px = int(board_width_inches * DPI)
    board_height_px = int(board_height_inches * DPI)

    # --- Image Generation ---
    board_img = board.generateImage((board_width_px, board_height_px))

    # Create a full page white canvas
    page_width_px, page_height_px = CANVAS_SIZE_PX
    canvas = np.ones((page_height_px, page_width_px), dtype=np.uint8) * 255

    # Correctly calculate the position to center the board
    text_space_inches = 1.0
    board_area_height_px = page_height_px - int(text_space_inches * DPI)

    start_x = (page_width_px - board_width_px) // 2
    start_y = (board_area_height_px - board_height_px) // 2

    # If board is too tall for the allocated area, reduce text space
    if start_y < 0:
        board_area_height_px = page_height_px - int(0.5 * DPI)  # Reduce text space to 0.5 inches
        start_y = (board_area_height_px - board_height_px) // 2
        if start_y < 0:
            start_y = 0  # Place at top if still too big

    # Place the board image onto the canvas
    if start_y >= 0 and start_x >= 0:
        canvas[start_y : start_y + board_height_px, start_x : start_x + board_width_px] = board_img
    else:
        print("⚠️ WARNING: Board is larger than the available page area. It may be cropped.")
        canvas[max(0, start_y) : min(page_height_px, start_y + board_height_px),
               max(0, start_x) : min(page_width_px, start_x + board_width_px)] = board_img[
            max(0, -start_y) : min(board_height_px, page_height_px - start_y),
            max(0, -start_x) : min(board_width_px, page_width_px - start_x)]

    # --- PDF Creation ---
    fig_inches = (CANVAS_SIZE_PX[0] / DPI, CANVAS_SIZE_PX[1] / DPI)
    fig = plt.figure(figsize=fig_inches, dpi=DPI)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_axis_off()
    ax.imshow(canvas, cmap='gray', vmin=0, vmax=255)

    # **MODIFICATION: Create a single-line footer with the board's metadata**
    footer_text = (
        f"Dict: {aruco_dict_name} | "
        f"Size: {cols}x{rows} | "
        f"Checker: {checker_size_mm:.1f}mm | "
        f"Marker: {marker_size_mm:.1f}mm | "
        f"Print @ 100% Scale"
    )
    # Position the footer at the bottom of the page
    fig.text(0.5, 0.03, footer_text, ha='center', va='bottom', fontsize=7, family='monospace')

    try:
        plt.savefig(output_filename, dpi=DPI, format='pdf')
        print(f"\n✅ Successfully generated board: '{output_filename}'")
    except Exception as e:
        print(f"\n❌ Error saving file: {e}")

    plt.close(fig)

def main():
    parser = argparse.ArgumentParser(description='Generate ChArUco board patterns for camera calibration.')
    parser.add_argument('--rows', type=int, default=7, help='Number of rows of squares (default: 7)')
    parser.add_argument('--cols', type=int, default=5, help='Number of columns of squares (default: 5)')
    parser.add_argument('--checker-size', type=float, default=30.0, help='Size of a single checker square in mm (default: 30.0)')
    parser.add_argument('--marker-size', type=float, default=18.0, help='Size of the ArUco marker in mm (default: 18.0)')
    parser.add_argument('--dict', default='DICT_4X4_50', help='ArUco dictionary name (default: DICT_4X4_50)')
    parser.add_argument('--output', default='charuco_board.pdf', help='Output PDF filename (default: charuco_board.pdf)')
    args = parser.parse_args()

    if args.marker_size >= args.checker_size:
        print("❌ Error: --marker-size must be smaller than --checker-size.")
        return

    generate_charuco_board(
        rows=args.rows,
        cols=args.cols,
        checker_size_mm=args.checker_size,
        marker_size_mm=args.marker_size,
        aruco_dict_name=args.dict,
        output_filename=args.output
    )

if __name__ == '__main__':
    main()
