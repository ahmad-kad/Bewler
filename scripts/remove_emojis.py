#!/usr/bin/env python3
"""
Comprehensive Emoji Removal Script for URC Machiato 2026
Removes all emojis and special Unicode symbols from source files
"""

import re
import sys
from pathlib import Path


def remove_emojis_from_file(filepath):
    """Remove emojis and special Unicode symbols from a file."""
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            content = f.read()

        # Comprehensive emoji and symbol removal pattern
        emoji_pattern = re.compile(
            "["
            "\U0001F600-\U0001F64F"  # emoticons
            "\U0001F300-\U0001F5FF"  # symbols & pictographs
            "\U0001F680-\U0001F6FF"  # transport & map symbols
            "\U0001F1E0-\U0001F1FF"  # flags (iOS)
            "\U00002702-\U000027B0"
            "\U000024C2-\U0001F251"
            "\U0001f926-\U0001f937"
            "\U00010000-\U0010ffff"
            "\u2640-\u2642"
            "\u2600-\u2B55"
            "\u200d"
            "\u23cf"
            "\u23e9"
            "\u231a"
            "\ufe0f"  # dingbats
            "\u3030"
            "]+",
            flags=re.UNICODE
        )

        cleaned_content = emoji_pattern.sub('', content)

        if cleaned_content != content:
            with open(filepath, 'w', encoding='utf-8') as f:
                f.write(cleaned_content)
            return True
        return False

    except Exception as e:
        print(f"Error processing {filepath}: {e}", file=sys.stderr)
        return False


def main():
    """Main cleanup function."""
    project_root = Path(__file__).parent.parent

    # File types to clean
    extensions = ['*.py', '*.md', '*.rst', '*.txt', '*.yaml', '*.yml', '*.json', '*.sh', '*.js', '*.jsx']

    files_modified = 0
    files_processed = 0

    for ext in extensions:
        for filepath in project_root.rglob(ext):
            # Skip certain directories
            if any(skip in str(filepath) for skip in ['node_modules', 'ros2_ws', '.git', '__pycache__']):
                continue

            files_processed += 1
            if remove_emojis_from_file(filepath):
                files_modified += 1
                print(f"Cleaned: {filepath}")

    print(f"\nCleanup complete: {files_modified} files modified out of {files_processed} processed")


if __name__ == '__main__':
    main()
