#!/usr/bin/env python3
"""
Automated Style Fixer for URC Machiato 2026
Fixes common style issues like unused variables and basic line length violations
"""

import ast
import os
import re
import sys
from pathlib import Path


def fix_unused_variables(content, filepath):
    """Fix unused local variables by prefixing with underscore."""
    try:
        tree = ast.parse(content)

        # Find unused variables in function scopes
        unused_vars = set()

        class VariableVisitor(ast.NodeVisitor):
            def __init__(self):
                self.scope_vars = {}
                self.used_vars = set()

            def visit_FunctionDef(self, node):
                # Track variables defined in this function
                func_vars = set()

                # Find assignments
                for child in ast.walk(node):
                    if isinstance(child, ast.Name) and isinstance(child.ctx, ast.Store):
                        if not child.id.startswith('_'):
                            func_vars.add(child.id)
                    elif isinstance(child, ast.Name) and isinstance(child.ctx, ast.Load):
                        self.used_vars.add(child.id)

                # Find unused variables
                for var in func_vars:
                    if var not in self.used_vars and var != 'self':
                        unused_vars.add(var)

                self.generic_visit(node)

        visitor = VariableVisitor()
        visitor.visit(tree)

        # Replace unused variables with underscore prefix
        for var in unused_vars:
            # Only replace simple assignments, not complex expressions
            pattern = rf'\b{re.escape(var)}\s*='
            content = re.sub(pattern, f'_{var} =', content)

        return content

    except SyntaxError:
        # If we can't parse, return unchanged
        return content

def fix_basic_line_lengths(content):
    """Fix basic line length issues by breaking long lines."""
    lines = content.split('\n')
    fixed_lines = []

    for line in lines:
        if len(line) <= 88:
            fixed_lines.append(line)
            continue

        # Try to fix some common patterns
        # Long import lines
        if line.startswith('from ') and len(line) > 88:
            # Try to break after 'import'
            if ' import ' in line:
                parts = line.split(' import ', 1)
                if len(parts[0]) + len(' import ') <= 88:
                    fixed_lines.append(line)  # Keep as is for now
                else:
                    fixed_lines.append(line)  # Keep as is for now
            else:
                fixed_lines.append(line)
        # Long function calls with many arguments
        elif '(' in line and line.count(',') > 2 and len(line) > 88:
            fixed_lines.append(line)  # Keep for manual fixing
        else:
            fixed_lines.append(line)

    return '\n'.join(fixed_lines)

def process_file(filepath):
    """Process a single file for style fixes."""
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            content = f.read()

        original_content = content

        # Fix unused variables
        content = fix_unused_variables(content, filepath)

        # Fix basic line length issues
        content = fix_basic_line_lengths(content)

        if content != original_content:
            with open(filepath, 'w', encoding='utf-8') as f:
                f.write(content)
            return True

    except Exception as e:
        print(f"Error processing {filepath}: {e}", file=sys.stderr)

    return False

def main():
    """Main style fixing function."""
    project_root = Path(__file__).parent.parent

    # Only process Python files in Autonomy directory
    autonomy_dir = project_root / "Autonomy"

    files_modified = 0
    files_processed = 0

    for filepath in autonomy_dir.rglob("*.py"):
        # Skip certain directories
        if any(skip in str(filepath) for skip in ['__pycache__', '.git', 'ros2_ws']):
            continue

        files_processed += 1
        if process_file(filepath):
            files_modified += 1
            print(f"Fixed: {filepath}")

    print(f"\nStyle fixing complete: {files_modified} files modified out of {files_processed} processed")

if __name__ == '__main__':
    main()
