#!/usr/bin/env python3
"""
Comprehensive Style Fixer for URC Machiato 2026
Addresses all major style violations systematically
"""

import ast
import re
import sys
from pathlib import Path


def fix_line_lengths(content: str) -> str:
    """Fix line length violations by breaking long lines appropriately."""
    lines = content.split('\n')
    fixed_lines = []

    for line in lines:
        if len(line) <= 88:
            fixed_lines.append(line)
            continue

        # Skip lines that are already in multi-line constructs
        stripped = line.strip()
        if stripped.startswith(('"""', "'''", '#')) or not stripped:
            fixed_lines.append(line)
            continue

        # Handle function calls with many arguments
        if '(' in line and line.count(',') >= 3 and len(line) > 88:
            # Try to break after opening parenthesis
            if '(' in line and not line.strip().endswith(','):
                paren_idx = line.find('(')
                if paren_idx > 0 and paren_idx < 50:  # Reasonable function name length
                    indent = ' ' * (paren_idx + 1)
                    fixed_lines.append(line[:paren_idx + 1])
                    remaining = line[paren_idx + 1:]
                    # Break remaining args
                    if len(remaining) > 80:
                        args = remaining.rstrip(')').split(',')
                        if len(args) > 1:
                            for i, arg in enumerate(args[:-1]):
                                if i == 0:
                                    fixed_lines.append(f"{indent}{arg.strip()},")
                                else:
                                    fixed_lines.append(f"{indent}{arg.strip()},")
                            fixed_lines.append(f"{indent}{args[-1].strip()})")
                            continue
            fixed_lines.append(line)  # Keep as-is if can't break nicely
        else:
            fixed_lines.append(line)

    return '\n'.join(fixed_lines)


def fix_continuation_indentation(content: str) -> str:
    """Fix continuation line indentation (E128)."""
    lines = content.split('\n')
    fixed_lines = []

    for i, line in enumerate(lines):
        if i == 0:
            fixed_lines.append(line)
            continue

        # Check if this is a continuation line
        stripped = line.strip()
        if not stripped or stripped.startswith('#'):
            fixed_lines.append(line)
            continue

        # Look for backslash continuation or implicit continuation
        prev_line = lines[i - 1].rstrip()
        if prev_line.endswith('\\') or (prev_line and not prev_line.rstrip().endswith((':', '(', '[', '{')) and
                                        not stripped.startswith((')', ']', '}', 'elif ', 'else:', 'except ', 'finally:'))):
            # This might be a continuation - check indentation
            if line.startswith(' ') and not line.startswith('    '):  # Not properly indented
                # Calculate proper indentation based on previous line
                prev_indent = len(prev_line) - len(prev_line.lstrip())
                if prev_line.strip().endswith(('(', '[', '{')):
                    new_indent = prev_indent + 4
                else:
                    new_indent = prev_indent + 4  # Standard continuation indent
                fixed_lines.append(' ' * new_indent + stripped)
            else:
                fixed_lines.append(line)
        else:
            fixed_lines.append(line)

    return '\n'.join(fixed_lines)


def fix_blank_lines(content: str) -> str:
    """Fix blank line spacing issues (E302, E305)."""
    lines = content.split('\n')
    fixed_lines = []

    i = 0
    while i < len(lines):
        line = lines[i]
        fixed_lines.append(line)

        # Check for class or function definitions
        if line.strip().startswith(('class ', 'def ')) and i + 1 < len(lines):
            # Ensure exactly 2 blank lines before class/function (except at start of file)
            blank_count = 0
            j = i - 1
            while j >= 0 and not lines[j].strip():
                blank_count += 1
                j -= 1

            # Remove extra blank lines before definitions
            if blank_count > 2 and i > 0:
                # Remove extra blank lines
                del fixed_lines[-blank_count:-2]

            # Add missing blank lines if needed
            elif blank_count < 2 and i > 0 and not lines[i - 1].strip().startswith(('class ', 'def ')):
                # Add blank lines before class/function
                while len([l for l in fixed_lines[-3:] if not l.strip()]) < 2:
                    fixed_lines.insert(-1, '')

        i += 1

    return '\n'.join(fixed_lines)


def fix_unused_variables(content: str, filepath: str) -> str:
    """Fix unused local variables by prefixing with underscore."""
    try:
        tree = ast.parse(content)

        # Find unused variables in function scopes
        unused_vars = set()

        class VariableVisitor(ast.NodeVisitor):
            def __init__(self):
                self.scope_vars = {}
                self.used_vars = set()
                self.current_function = None

            def visit_FunctionDef(self, node):
                old_function = self.current_function
                self.current_function = node.name

                # Reset for new function
                old_scope = self.scope_vars.copy()
                old_used = self.used_vars.copy()

                self.scope_vars.clear()
                self.used_vars.clear()

                # Find assignments in this function
                for child in ast.walk(node):
                    if isinstance(child, ast.Name) and isinstance(child.ctx, ast.Store):
                        if not child.id.startswith('_') and child.id != 'self':
                            self.scope_vars.add(child.id)
                    elif isinstance(child, ast.Name) and isinstance(child.ctx, ast.Load):
                        self.used_vars.add(child.id)

                # Find unused variables (only if they appear to be truly unused)
                for var in self.scope_vars:
                    if var not in self.used_vars:
                        # Double-check by searching in the source
                        if f'{var}(' not in content and f' {var} ' not in content and f'={var}' not in content:
                            unused_vars.add(var)

                # Restore previous scope
                self.scope_vars = old_scope
                self.used_vars = old_used
                self.current_function = old_function

                self.generic_visit(node)

        visitor = VariableVisitor()
        visitor.visit(tree)

        # Replace unused variables with underscore prefix
        for var in unused_vars:
            # Use word boundaries to avoid partial matches
            pattern = rf'\b{re.escape(var)}\s*='
            content = re.sub(pattern, f'_{var} =', content)

        return content

    except SyntaxError:
        return content


def process_file(filepath: str) -> bool:
    """Process a single file for all style fixes."""
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            content = f.read()

        original_content = content

        # Apply fixes in order
        content = fix_unused_variables(content, filepath)
        content = fix_line_lengths(content)
        content = fix_continuation_indentation(content)
        content = fix_blank_lines(content)

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

    # Only process Python files, excluding certain directories
    exclude_patterns = [
        'node_modules', '__pycache__', '.git', 'ros2_ws',
        'docs/_build', 'frontend/node_modules'
    ]

    files_modified = 0
    files_processed = 0

    for filepath in project_root.rglob("*.py"):
        # Skip excluded directories
        if any(excl in str(filepath) for excl in exclude_patterns):
            continue

        files_processed += 1
        if process_file(str(filepath)):
            files_modified += 1
            print(f"Fixed: {filepath}")

    print(f"\nStyle fixing complete: {files_modified} files modified out of {files_processed} processed")


if __name__ == '__main__':
    main()
