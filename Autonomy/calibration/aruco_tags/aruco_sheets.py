
#!/usr/bin/env python3
"""
ArUco Tag Sheet Generator for Environment Setup - URC 2026

Generates multi-tag ArUco sheets for environment setup and navigation markers.
These sheets are used for:
- Navigation landmark arrays
- Equipment/service area marker sets
- Autonomous typing keyboard layouts
- General environmental fiducial systems

PRINTABLE AREA CONSTRAINTS (Based on template_silhouette.png):
- Page size: US Letter (11" x 8.5", 3301x2550px @ 300 DPI)
- Safe margins: 0.75" on all sides (225px @ 300 DPI)
- Usable printable area: 9.5" x 7.0" (2851x2100px)
- This script ensures ALL tags fit within these safe printable boundaries

RECOMMENDED TAG SIZES:
- Small tags (1-2cm): Multiple per page (42x 2cm tags or 90x 1cm tags)
- Medium tags (5-10cm): 2-4 tags per page
- Large tags (15-20cm): 1 tag per page

NOTE: These ArUco tag sheets are NOT used for camera calibration.
Use ChArUco boards (charuco_board/generate_chessboard.py) for camera
intrinsics and hand-eye calibration.

Usage:
    python aruco_sheets.py --output navigation_sheet.pdf
    python aruco_sheets.py -o custom_tags.pdf
"""

import argparse
from datetime import datetime

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


def generate_tag_image(marker_id, size_px, aruco_dict):
    """Generate a single ArUco tag as numpy array (RGB)."""
    img_gray = cv2.aruco.generateImageMarker(aruco_dict, marker_id, size_px)
    img_rgb = cv2.cvtColor(img_gray, cv2.COLOR_GRAY2RGB)
    return img_rgb


def create_page_with_tags(tag_specs, start_id, dpi=300, landscape=False):
    """
    Create a single page with multiple tags for STANDARD LETTER paper.

    tag_specs: list of (size_cm, count) tuples
    start_id: starting marker ID
    landscape: if True, use landscape orientation
    Returns: (PIL_image, next_id)

    ✅ Respects template_silhouette.png constraints:
    - Page: US Letter (11" × 8.5", 3301×2550px @ 300 DPI)
    - Safe margins: 0.75" (225px)
    - Usable area: 9.5" × 7.0" (2851×2100px)
    - Layout: Fills available space efficiently (no centering gaps)
    """
    # Desired canvas size: 3301x2550 pixels at 300 DPI (US Letter)
    DESIRED_CANVAS_PX = (3301, 2550)
    CM_PER_INCH = 2.54

    # Safe printable area margins (0.75" on all sides based on template_silhouette.png)
    PRINT_MARGIN_INCHES = 0.75

    # Use fixed pixel dimensions for exact canvas size
    if landscape:
        page_width_px = DESIRED_CANVAS_PX[0]  # 3301
        page_height_px = DESIRED_CANVAS_PX[1]  # 2550
    else:
        # Portrait: swap dimensions
        page_width_px = DESIRED_CANVAS_PX[1]  # 2550
        page_height_px = DESIRED_CANVAS_PX[0]  # 3301

    # Calculate equivalent inches for margin calculations
    page_width_inches = page_width_px / dpi
    page_height_inches = page_height_px / dpi

    # Convert margin to pixels
    margin = int(PRINT_MARGIN_INCHES * dpi)

    # Create white canvas
    canvas = np.ones((page_height_px, page_width_px, 3), dtype=np.uint8) * 255

    # Get ArUco dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    num_markers = aruco_dict.bytesList.shape[0]

    current_id = start_id
    dpi_val = dpi

    # Calculate usable area (within safe margins)
    usable_width_px = page_width_px - 2 * margin
    usable_height_px = page_height_px - 2 * margin

    # Check if any tags are too large
    max_tag_size_cm = max(size_cm for size_cm, _ in tag_specs)
    max_tag_size_inches = max_tag_size_cm / CM_PER_INCH
    max_tag_size_px = int(max_tag_size_inches * dpi)

    if max_tag_size_px > usable_width_px or max_tag_size_px > usable_height_px:
        print(f"  ⚠️  WARNING: {max_tag_size_cm}cm tag "
              f"({max_tag_size_inches:.1f}\") too large for printable area!")
        margin_inches = PRINT_MARGIN_INCHES
        width_inches = page_width_inches - 2 * margin_inches
        height_inches = page_height_inches - 2 * margin_inches
        print(f"     Printable area: {usable_width_px}px x {usable_height_px}px "
              f"({width_inches:.1f}\" x {height_inches:.1f}\")")
        print("     Tag will be clipped. Consider using smaller tags.")

    # Start from top-left of usable area (no centering)
    current_y = margin

    for size_cm, count in tag_specs:
        tag_size_inches = size_cm / CM_PER_INCH
        tag_size_px = int(tag_size_inches * dpi_val)

        # Minimal padding between tags (0.15 inch = 45px at 300dpi)
        tag_padding = int(0.15 * dpi_val)

        # How many fit horizontally in the usable area
        tags_per_row = max(1, usable_width_px // (tag_size_px + tag_padding))

        # Generate tags row by row
        for tag_idx in range(count):
            if current_id >= num_markers:
                current_id = 0  # Wrap around if needed

            row_pos = tag_idx // tags_per_row
            col_pos = tag_idx % tags_per_row

            x = margin + col_pos * (tag_size_px + tag_padding)
            y = current_y + row_pos * (tag_size_px + tag_padding)

            # Check if we're exceeding the safe printable area
            if y + tag_size_px > page_height_px - margin:
                remaining_tags = count - tag_idx
                print(f"  ⚠️  WARNING: Not enough vertical space for all tags. "
                      f"{remaining_tags} tags will be omitted.")
                break

            # Check horizontal bounds
            if x + tag_size_px > page_width_px - margin:
                print("  ⚠️  WARNING: Tag exceeds horizontal bounds.")
                continue

            # Generate tag
            tag_img = generate_tag_image(current_id, tag_size_px, aruco_dict)

            # Place on canvas
            canvas[y : y + tag_size_px, x : x + tag_size_px] = tag_img

            # Draw cutlines around tag
            pil_img = Image.fromarray(canvas)
            draw = ImageDraw.Draw(pil_img)
            cutline_color = (200, 200, 200)  # Light gray
            cutline_width = 2
            # Rectangle around tag
            draw.rectangle(
                [(x - 5, y - 5), (x + tag_size_px + 5, y + tag_size_px + 5)],
                outline=cutline_color,
                width=cutline_width
            )
            canvas = np.array(pil_img)

            # Draw ID label below tag
            label_y = y + tag_size_px + 5
            draw = ImageDraw.Draw(Image.fromarray(canvas))
            try:
                font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf"
                font = ImageFont.truetype(font_path, 12)
            except (IOError, OSError):
                font = ImageFont.load_default()

            label_text = f"ID:{current_id} ({size_cm}cm)"
            # Convert back to numpy to draw, then back to array
            pil_img = Image.fromarray(canvas)
            draw = ImageDraw.Draw(pil_img)
            draw.text((x, label_y), label_text, fill=(0, 0, 0), font=font)
            canvas = np.array(pil_img)

            current_id += 1

        # Move to next row section (after all rows for this tag size)
        total_rows = (count + tags_per_row - 1) // tags_per_row
        section_gap = int(0.2 * dpi)  # Small vertical gap between sections
        current_y += total_rows * (tag_size_px + tag_padding) + section_gap

    # Add metadata at the bottom of the page (within safe area)
    pil_img = Image.fromarray(canvas)
    draw = ImageDraw.Draw(pil_img)

    # Try to load a better font, fallback to default
    try:
        font = ImageFont.truetype(font_path, 10)
    except (IOError, OSError):
        font = ImageFont.load_default()

    # Metadata information
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    metadata_lines = [
        f"ArUco Tag Sheet - Generated {timestamp}",
        f"Dictionary: DICT_4X4_50, DPI: {dpi}",
        f"Tags: {', '.join([f'{count}x{size}cm' for size, count in tag_specs])}",
        f"Page size: {page_width_inches:.1f}\" x {page_height_inches:.1f}\" "
        f"({page_width_px}x{page_height_px}px), Margins: {PRINT_MARGIN_INCHES:.2f}\"",
        f"IDs: {start_id} to {current_id-1}"
    ]

    # Position metadata at bottom of page within safe area
    metadata_y = page_height_px - margin - 10
    line_height = 12

    for line in reversed(metadata_lines):
        draw.text((margin + 10, metadata_y), line, fill=(100, 100, 100), font=font)
        metadata_y -= line_height

    canvas = np.array(pil_img)

    pil_image = Image.fromarray(canvas, mode='RGB')
    return pil_image, current_id


def create_page_with_tags_large_format(tag_specs, start_id, dpi=300):
    """
    Create a single page with tags for LARGE FORMAT roll paper (36" × 44").

    tag_specs: list of (size_cm, count) tuples
    start_id: starting marker ID
    Returns: (PIL_image, next_id)

    ✅ Optimized for 36" × 44" roll paper:
    - Arranges tags in efficient rows (fills width)
    - Minimal margins (0.5")
    - Maximum utilization of paper width
    - Typically used for large navigation posts (10-20cm tags)
    """
    PAPER_WIDTH_INCHES = 36.0
    PAPER_HEIGHT_INCHES = 44.0
    CM_PER_INCH = 2.54
    PRINT_MARGIN_INCHES = 0.5  # Minimal margin for roll paper

    # Convert to pixels @ 300 DPI
    page_width_px = int(PAPER_WIDTH_INCHES * dpi)
    page_height_px = int(PAPER_HEIGHT_INCHES * dpi)
    margin = int(PRINT_MARGIN_INCHES * dpi)

    # Usable area
    usable_width_px = page_width_px - 2 * margin

    # Create white canvas
    canvas = np.ones((page_height_px, page_width_px, 3), dtype=np.uint8) * 255

    # Get ArUco dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    num_markers = aruco_dict.bytesList.shape[0]

    current_id = start_id
    current_y = margin

    for size_cm, count in tag_specs:
        tag_size_inches = size_cm / CM_PER_INCH
        tag_size_px = int(tag_size_inches * dpi)

        # Spacing between tags
        tag_spacing = int(0.3 * dpi)  # 0.3" spacing

        # Fill row: calculate how many tags fit across the width
        tags_per_row = max(1, usable_width_px // (tag_size_px + tag_spacing))

        # Calculate total layout
        total_rows_needed = (count + tags_per_row - 1) // tags_per_row
        layout_width = tags_per_row * tag_size_px + max(0, (tags_per_row - 1) * tag_spacing)

        # Center horizontally
        start_x = margin + (usable_width_px - layout_width) // 2

        # Generate tags row by row
        for tag_idx in range(count):
            if current_id >= num_markers:
                current_id = 0  # Wrap around if needed

            row_pos = tag_idx // tags_per_row
            col_pos = tag_idx % tags_per_row

            x = start_x + col_pos * (tag_size_px + tag_spacing)
            y = current_y + row_pos * (tag_size_px + tag_spacing)

            # Check if we're exceeding the page height
            if y + tag_size_px > page_height_px - margin:
                print(
                    f"  ⚠️  WARNING: Insufficient vertical space for all tags. "
                    f"{count - tag_idx} tags will be omitted."
                )
                break

            # Generate tag
            tag_img = generate_tag_image(current_id, tag_size_px, aruco_dict)

            # Place on canvas
            canvas[y : y + tag_size_px, x : x + tag_size_px] = tag_img

            # Draw cutlines around tag
            pil_img = Image.fromarray(canvas)
            draw = ImageDraw.Draw(pil_img)
            cutline_color = (200, 200, 200)
            cutline_width = 3
            draw.rectangle(
                [(x - 8, y - 8), (x + tag_size_px + 8, y + tag_size_px + 8)],
                outline=cutline_color,
                width=cutline_width
            )
            canvas = np.array(pil_img)

            # Draw ID label
            label_y = y + tag_size_px + 10
            pil_img = Image.fromarray(canvas)
            draw = ImageDraw.Draw(pil_img)
            try:
                font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 16)
            except (IOError, OSError):
                font = ImageFont.load_default()

            label_text = f"ID:{current_id} ({size_cm}cm)"
            draw.text((x, label_y), label_text, fill=(0, 0, 0), font=font)
            canvas = np.array(pil_img)

            current_id += 1

        # Move to next row section
        current_y += total_rows_needed * (tag_size_px + tag_spacing) + int(0.5 * dpi)

    # Add metadata
    pil_img = Image.fromarray(canvas)
    draw = ImageDraw.Draw(pil_img)

    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 12)
    except (IOError, OSError):
        font = ImageFont.load_default()

    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    metadata_lines = [
        f"ArUco Large Format (Roll Paper) - {timestamp}",
        f"Paper: 36\" × 44\" | Dictionary: DICT_4X4_50 | DPI: {dpi}",
        f"Tags: {', '.join([f'{count}x{size}cm' for size, count in tag_specs])}",
        f"IDs: {start_id} to {current_id-1}"
    ]

    metadata_y = page_height_px - margin - 20
    line_height = 16

    for line in reversed(metadata_lines):
        draw.text((margin + 20, metadata_y), line, fill=(100, 100, 100), font=font)
        metadata_y -= line_height

    canvas = np.array(pil_img)
    pil_image = Image.fromarray(canvas, mode='RGB')
    return pil_image, current_id


def create_merged_template_with_tags(tag_specs, start_id, template_image_path, dpi=300):
    """
    Merge small ArUco tags with template_silhouette.png as registration marks.

    tag_specs: list of (size_cm, count) tuples
    start_id: starting marker ID
    template_image_path: path to template_silhouette.png
    Returns: (PIL_image, next_id)

    ✅ Overlays ArUco tags on template_silhouette.png:
    - Template: US Letter (11" × 8.5")
    - Tags: Overlaid as registration/fiducial marks
    - Layout: Corner and edge positions for alignment
    """
    CM_PER_INCH = 2.54
    PRINT_MARGIN_INCHES = 0.75

    # Load template image
    try:
        template_img = Image.open(template_image_path)
        print(f"  ✅ Loaded template: {template_image_path}")
        print(f"     Template size: {template_img.size}")
    except FileNotFoundError:
        print(f"  ❌ Template not found: {template_image_path}")
        return None, start_id

    # Convert template to RGB if needed
    if template_img.mode != 'RGB':
        template_img = template_img.convert('RGB')

    # Get template dimensions
    template_width_px, template_height_px = template_img.size

    # Convert to numpy for manipulation
    canvas = np.array(template_img)

    # Get ArUco dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    num_markers = aruco_dict.bytesList.shape[0]

    current_id = start_id
    margin = int(PRINT_MARGIN_INCHES * dpi)

    # Define corner positions for registration marks
    # corners = [
    #     ("top-left", margin, margin),
    #     ("top-right", template_width_px - margin, margin),
    #     ("bottom-left", margin, template_height_px - margin),
    #     ("bottom-right", template_width_px - margin, template_height_px - margin),
    # ]

    # Track positions for main tags
    start_x = margin + int(1.0 * dpi)  # Offset from corner marks
    start_y = margin + int(1.5 * dpi)
    current_y = start_y

    usable_width_px = template_width_px - 2 * margin - int(1.0 * dpi)

    for size_cm, count in tag_specs:
        tag_size_inches = size_cm / CM_PER_INCH
        tag_size_px = int(tag_size_inches * dpi)
        tag_padding = int(0.15 * dpi)

        # How many fit horizontally
        tags_per_row = max(1, usable_width_px // (tag_size_px + tag_padding))

        # Generate tags row by row
        for tag_idx in range(count):
            if current_id >= num_markers:
                current_id = 0

            row_pos = tag_idx // tags_per_row
            col_pos = tag_idx % tags_per_row

            x = start_x + col_pos * (tag_size_px + tag_padding)
            y = current_y + row_pos * (tag_size_px + tag_padding)

            # Check bounds
            if y + tag_size_px > template_height_px - margin:
                print(f"  ⚠️  Not enough space for all tags. {count - tag_idx} omitted.")
                break

            if x + tag_size_px > template_width_px - margin:
                continue

            # Generate tag
            tag_img = generate_tag_image(current_id, tag_size_px, aruco_dict)

            # Place on canvas
            canvas[y : y + tag_size_px, x : x + tag_size_px] = tag_img

            # Draw cutline
            pil_img = Image.fromarray(canvas)
            draw = ImageDraw.Draw(pil_img)
            draw.rectangle(
                [(x - 3, y - 3), (x + tag_size_px + 3, y + tag_size_px + 3)],
                outline=(200, 200, 200),
                width=2
            )
            canvas = np.array(pil_img)

            # Draw ID label
            label_y = y + tag_size_px + 3
            pil_img = Image.fromarray(canvas)
            draw = ImageDraw.Draw(pil_img)
            try:
                font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 10)
            except (IOError, OSError):
                font = ImageFont.load_default()

            label_text = f"ID:{current_id}"
            draw.text((x, label_y), label_text, fill=(0, 0, 0), font=font)
            canvas = np.array(pil_img)

            current_id += 1

        # Move to next row section
        total_rows = (count + tags_per_row - 1) // tags_per_row
        current_y += total_rows * (tag_size_px + tag_padding) + int(0.2 * dpi)

    # Add metadata
    pil_img = Image.fromarray(canvas)
    draw = ImageDraw.Draw(pil_img)

    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 9)
    except (IOError, OSError):
        font = ImageFont.load_default()

    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    metadata = f"Template + ArUco Tags | Generated {timestamp} | IDs: {start_id} to {current_id-1}"

    draw.text((margin, template_height_px - margin - 20), metadata, fill=(100, 100, 100), font=font)

    canvas = np.array(pil_img)
    return Image.fromarray(canvas, mode='RGB'), current_id


def generate_merged_template_pdf(
    tag_specs,
    output_filename="aruco_template_merged.pdf",
    template_path="template_silhouette.png"
):
    """
    Generate PDF with ArUco tags merged onto template_silhouette.png.

    Creates registration marks on the template image.
    """
    print("--- ArUco Template Merger ---")
    print(f"Template: {template_path}")

    pil_img, current_id = create_merged_template_with_tags(
        tag_specs,
        start_id=0,
        template_image_path=template_path,
        dpi=300
    )

    if pil_img is None:
        print("❌ Failed to merge template with tags")
        return

    total_tags = sum(count for _, count in tag_specs)

    try:
        # Save as PDF
        pil_img.save(output_filename, format='PDF', dpi=(300, 300))
        print(f"\n✅ Successfully generated: '{output_filename}'")
        print(f"   Tags: {total_tags} (IDs 0 to {current_id-1})")
        print("   Resolution: 300 DPI")
        print("   Format: A4 Letter with overlaid registration marks")
    except Exception as e:
        print(f"\n❌ Error saving file: {e}")


def generate_sheets(specs_list, output_filename="aruco_tags.pdf"):
    """
    Generate multi-page PDF with grouped tag layouts for STANDARD LETTER format.

    specs_list: list of (layout_description, [(size_cm, count), ...], landscape)
    """
    print("--- ArUco Tag Sheet Generator (Standard Letter Format) ---")
    print(f"OpenCV version: {cv2.__version__}")

    pages = []
    current_id = 0

    for layout_name, tag_specs, landscape in specs_list:
        print(f"\nGenerating: {layout_name}")
        start_id_for_page = current_id
        if landscape:
            pil_img, next_id = create_page_with_tags(tag_specs, current_id, landscape=landscape)
        else:
            pil_img, next_id = create_page_with_tags(tag_specs, current_id)
        pages.append(pil_img)

        # Calculate stats
        total_tags = sum(count for _, count in tag_specs)
        orientation = "Landscape" if landscape else "Portrait"
        end_id = next_id - 1
        print(f"  → {total_tags} tags (IDs {start_id_for_page} to {end_id}) - {orientation}")
        current_id = next_id

    # Save multi-page PDF
    try:
        pages[0].save(
            output_filename,
            format='PDF',
            save_all=True,
            append_images=pages[1:] if len(pages) > 1 else []
        )
        print(f"\n✅ Successfully generated: '{output_filename}'")
        print(f"   Total pages: {len(pages)}")
    except Exception as e:
        print(f"\n❌ Error saving file: {e}")


def generate_sheets_large_format(specs_list, output_filename="aruco_tags_large.pdf"):
    """
    Generate multi-page PDF with grouped tag layouts for LARGE FORMAT roll paper (36" × 44").

    specs_list: list of (layout_description, [(size_cm, count), ...])
    Tags are arranged in rows filling the width of the paper.
    """
    print("--- ArUco Tag Sheet Generator (Large Format Roll Paper - 36\" × 44\") ---")
    print(f"OpenCV version: {cv2.__version__}")

    pages = []
    current_id = 0

    for layout_name, tag_specs in specs_list:
        print(f"\nGenerating: {layout_name}")
        start_id_for_page = current_id
        pil_img, next_id = create_page_with_tags_large_format(tag_specs, current_id)
        pages.append(pil_img)

        # Calculate stats
        total_tags = sum(count for _, count in tag_specs)
        end_id = next_id - 1
        print(f"  → {total_tags} tags (IDs {start_id_for_page} to {end_id}) - Row layout")
        current_id = next_id

    # Save multi-page PDF
    try:
        pages[0].save(
            output_filename,
            format='PDF',
            save_all=True,
            append_images=pages[1:] if len(pages) > 1 else []
        )
        print(f"\n✅ Successfully generated: '{output_filename}'")
        print(f"   Total pages: {len(pages)}")
    except Exception as e:
        print(f"\n❌ Error saving file: {e}")


def generate_tags_as_pngs(tag_specs, output_dir="aruco_tags_png", dpi=300):
    """
    Generate individual ArUco tags as PNG images.

    Useful for creating separate PNG files for layering and compositing.
    """
    import os

    # Create output directory
    os.makedirs(output_dir, exist_ok=True)

    print("--- Generating ArUco Tags as PNG ---")
    print(f"Output directory: {output_dir}")

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    num_markers = aruco_dict.bytesList.shape[0]

    current_id = 0
    CM_PER_INCH = 2.54
    generated_count = 0

    for size_cm, count in tag_specs:
        tag_size_inches = size_cm / CM_PER_INCH
        tag_size_px = int(tag_size_inches * dpi)

        print(f"\nGenerating {count}× {size_cm}cm tags ({tag_size_px}×{tag_size_px}px)")

        for tag_idx in range(count):
            if current_id >= num_markers:
                current_id = 0

            # Generate tag image
            tag_img_gray = cv2.aruco.generateImageMarker(aruco_dict, current_id, tag_size_px)
            tag_img_rgb = cv2.cvtColor(tag_img_gray, cv2.COLOR_GRAY2RGB)

            # Create PIL image with white border
            border = 10
            canvas = Image.new('RGB', (tag_size_px + 2*border, tag_size_px + 2*border), 'white')
            tag_pil = Image.fromarray(tag_img_rgb)
            canvas.paste(tag_pil, (border, border))

            # Add ID label
            draw = ImageDraw.Draw(canvas)
            try:
                font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 14)
            except (IOError, OSError):
                font = ImageFont.load_default()

            label_text = f"ID:{current_id}\n{size_cm}cm"
            text_y = tag_size_px + border + 5
            draw.text((border, text_y), label_text, fill=(0, 0, 0), font=font)

            # Save PNG
            filename = f"{output_dir}/aruco_id_{current_id:03d}_{size_cm}cm.png"
            canvas.save(filename, dpi=(dpi, dpi))
            generated_count += 1

            if (tag_idx + 1) % max(1, count // 4) == 0 or tag_idx == count - 1:
                print(f"  ✓ Generated {tag_idx + 1}/{count} tags")

            current_id += 1

    print(f"\n✅ Successfully generated {generated_count} PNG files")
    print(f"   Location: {os.path.abspath(output_dir)}")


def main():
    parser = argparse.ArgumentParser(
        description='Generate multi-tag ArUco sheets for printing.',
        usage='%(prog)s [--format {small,large}] [--template] [--png] [-o OUTPUT]'
    )
    parser.add_argument(
        '--format',
        choices=['small', 'large'],
        default='small',
        help='Format: small (11"×8.5" letter) or large (36"×44" roll paper) [default: small]'
    )
    parser.add_argument(
        '--template',
        action='store_true',
        help='Merge small ArUco tags with template_silhouette.png as registration marks'
    )
    parser.add_argument(
        '--png',
        action='store_true',
        help='Generate individual ArUco tags as PNG files (for layering/compositing)'
    )
    parser.add_argument(
        '-o', '--output',
        default=None,
        help='Output PDF filename [default: aruco_sheets.pdf (small) or aruco_sheets_large.pdf (large)]'
    )
    args = parser.parse_args()

    # Set default output filename based on format and mode
    if args.output is None:
        if args.template:
            args.output = 'aruco_template_merged.pdf'
        else:
            args.output = 'aruco_sheets.pdf' if args.format == 'small' else 'aruco_sheets_large.pdf'

    print("\n📋 ArUco Sheet Generation Configuration:")

    # PNG export mode
    if args.png:
        print("   Mode: PNG EXPORT")
        print("   Output: Individual PNG files for each tag")
        print("   Use: Layering, compositing, or custom layouts\n")

        tag_specs = [
            (2, 42),   # 42× 2cm tags
            (1, 90),   # 90× 1cm tags
        ]

        generate_tags_as_pngs(tag_specs, output_dir="aruco_tags_png")

    # Template merge mode
    elif args.template:
        print("   Mode: TEMPLATE MERGE")
        print("   Base: template_silhouette.png")
        print("   Overlay: Small ArUco tags as registration marks")
        print("   Output: Single PDF with merged image\n")

        tag_specs = [
            (2, 42),   # 42× 2cm tags
            (1, 90),   # 90× 1cm tags
        ]

        generate_merged_template_pdf(tag_specs, args.output)

    elif args.format == 'small':
        print("   Format: SMALL - Standard Letter Paper")
        print("   Page size: 11\" × 8.5\" (US Letter)")
        print("   Printable area: 9.5\" × 7.0\" (within template_silhouette.png margins)")
        print("   Safe margins: 0.75\" on all sides")
        print("   DPI: 300")
        print("   Recommended single tag sizes: 1-7cm for multiple tags\n")

        # Define small format layouts
        layouts = [
            ("Small tags (keyboard USB corners + extras) - 2cm", [
                (2, 42),  # 42x 2cm tags for keyboard corners + extras

            ], True),
            ("Small tags (keyboard USB corners + extras) - 1cm", [

                (1, 90),  # 90x 1cm tags for USB slot corners + extras
            ], True),
            ("Medium tags (navigation markers) - 10cm", [
                (10, 2),  # 2x 10cm tags per page (fits 2 per row)
            ], True),
            ("Medium tags (navigation markers) - 10cm - Page 2", [
                (10, 2),
            ], True),
            ("Large tags (navigation posts) - 15cm", [
                (15, 1),  # 1x 15cm tag per page
            ], True),
            ("Large tags (navigation posts) - 15cm - Page 2", [
                (15, 1),
            ], True),
        ]

        generate_sheets(layouts, args.output)

    else:  # large format
        print("   Format: LARGE - Roll Paper (36\" × 44\")")
        print("   Page size: 36\" × 44\" (big printer roll)")
        print("   Margins: 0.5\" on all sides")
        print("   Layout: Tags arranged in efficient rows (fills width)")
        print("   DPI: 300")
        print("   Recommended tag sizes: 10-20cm (large navigation posts)\n")

        # Define large format layouts - tags listed as (size_cm, count)
        # These will be arranged in rows filling the 36" width
        layouts = [
            ("Large navigation posts - 20cm", [
                (20, 8),  # 8× 20cm tags = ~8 tags per row (20cm × 8 + spacing = ~180cm / 36")
            ]),
            ("Large navigation posts - 15cm", [
                (15, 12),  # 12× 15cm tags = more per row (15cm × 12 + spacing = ~180cm / 36")
            ]),
        ]

        generate_sheets_large_format(layouts, args.output)


if __name__ == '__main__':
    main()
