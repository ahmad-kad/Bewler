#!/usr/bin/env python3
"""
Markdown to RST Converter for URC Machiato Documentation

Converts existing markdown documentation to reStructuredText format
for integration with Sphinx documentation system.
"""

import os
import re
from pathlib import Path
from typing import Dict, List


class MarkdownToRSTConverter:
    """Converts Markdown files to reStructuredText format."""

    def __init__(self):
        self.conversion_patterns = {
            # Headers
            r'^#{6}\s+(.+)$': r'^^^^^\n\1\n^^^^^',  # H6
            r'^#{5}\s+(.+)$': r'"""""^\n\1\n""""^',  # H5
            r'^#{4}\s+(.+)$': r'"""\n\1\n"""',      # H4
            r'^#{3}\s+(.+)$': r'""\n\1\n""',        # H3
            r'^#{2}\s+(.+)$': r'-\n\1\n-',          # H2
            r'^#{1}\s+(.+)$': r'=\n\1\n=',          # H1

            # Code blocks (fenced)
            r'```(\w+)?\n(.*?)\n```': self._convert_code_block,

            # Inline code
            r'`([^`]+)`': r'``\1``',

            # Links
            r'\[([^\]]+)\]\(([^)]+)\)': r'`\1 <\2>`_',

            # Bold
            r'\*\*([^*]+)\*\*': r'**\1**',

            # Italic
            r'\*([^*]+)\*': r'*\1*',

            # Lists
            r'^\s*-\s+(.+)$': r'* \1',
            r'^\s*\d+\.\s+(.+)$': r'# \1',

            # Tables (basic conversion)
            r'\|(.+)\|': self._convert_table_row,
        }

    def _convert_code_block(self, match):
        """Convert fenced code blocks to RST format."""
        lang = match.group(1) or ''
        code = match.group(2)
        if lang:
            return f'.. code-block:: {lang}\n\n   {code.replace(chr(10), chr(10) + "   ")}'
        else:
            return f'::\n\n   {code.replace(chr(10), chr(10) + "   ")}'

    def _convert_table_row(self, match):
        """Convert markdown table rows to RST format."""
        # This is a simplified conversion - full table conversion would be more complex
        content = match.group(1)
        return content  # For now, just remove the pipes

    def convert_file(self, md_file: Path, rst_file: Path) -> None:
        """Convert a single markdown file to RST format."""
        with open(md_file, 'r', encoding='utf-8') as f:
            content = f.read()

        # Apply conversions
        rst_content = content
        for pattern, replacement in self.conversion_patterns.items():
            if callable(replacement):
                rst_content = re.sub(pattern, replacement, rst_content, flags=re.MULTILINE | re.DOTALL)
            else:
                rst_content = re.sub(pattern, replacement, rst_content, flags=re.MULTILINE)

        # Add RST header
        title = md_file.stem.replace('_', ' ').title()
        rst_header = f'{title}\n{"=" * len(title)}\n\n'

        # Write RST file
        with open(rst_file, 'w', encoding='utf-8') as f:
            f.write(rst_header)
            f.write(rst_content)

        print(f"Converted {md_file} -> {rst_file}")

    def convert_directory(self, source_dir: Path, target_dir: Path) -> None:
        """Convert all markdown files in a directory tree."""
        for md_file in source_dir.rglob('*.md'):
            # Create corresponding RST path
            relative_path = md_file.relative_to(source_dir)
            rst_file = target_dir / relative_path.with_suffix('.rst')

            # Create target directory if needed
            rst_file.parent.mkdir(parents=True, exist_ok=True)

            # Skip if RST already exists and is newer
            if rst_file.exists() and rst_file.stat().st_mtime > md_file.stat().st_mtime:
                continue

            self.convert_file(md_file, rst_file)


def main():
    """Main conversion function."""
    converter = MarkdownToRSTConverter()

    # Define source and target directories
    project_root = Path(__file__).parent.parent
    source_dirs = [
        project_root / 'Autonomy' / 'docs',
        project_root / 'docs',
    ]

    target_dir = project_root / 'docs' / 'docs' / 'integrated'

    # Convert documentation
    target_dir.mkdir(parents=True, exist_ok=True)

    for source_dir in source_dirs:
        if source_dir.exists():
            print(f"Converting documentation from {source_dir}")
            converter.convert_directory(source_dir, target_dir)

    print("Documentation conversion complete!")


if __name__ == '__main__':
    main()
