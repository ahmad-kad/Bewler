#!/usr/bin/env python3
"""
Documentation Build Script - URC 2026

Automated script to build all documentation types for the URC 2026 project.
This script handles the complete documentation build process including
Python (Sphinx), C++ (Doxygen), and JavaScript/TypeScript (JSDoc) documentation.

Usage:
    python build_docs.py [--clean] [--serve] [--validate]

Options:
    --clean     Clean all build artifacts before building
    --serve     Serve documentation locally after building
    --validate  Run validation checks (links, doctests)
    --help      Show this help message

Environment Variables:
    DOCS_PORT  Port for local server (default: 8000)
    DOCS_HOST  Host for local server (default: localhost)
"""

import argparse
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path


class DocsBuilder:
    """Documentation builder for URC 2026 project."""

    def __init__(self, docs_dir: Path):
        self.docs_dir = docs_dir
        self.build_dir = docs_dir / '_build'
        self.source_dir = docs_dir

    def run_command(self, cmd: list, cwd: Path = None, check: bool = True) -> subprocess.CompletedProcess:
        """Run a command and return the result."""
        try:
            result = subprocess.run(
                cmd,
                cwd=cwd or self.docs_dir,
                capture_output=True,
                text=True,
                check=check
            )
            return result
        except subprocess.CalledProcessError as e:
            print(f"❌ Command failed: {' '.join(cmd)}")
            print(f"Error output: {e.stderr}")
            if check:
                sys.exit(1)
            return e

    def check_dependencies(self) -> bool:
        """Check if all required dependencies are installed."""
        print("🔍 Checking dependencies...")

        # Check Python dependencies
        try:
            import breathe
            import sphinx
            import sphinx_js
            print("✅ Python dependencies OK")
        except ImportError as e:
            print(f"❌ Missing Python dependency: {e}")
            print("Install with: pip install -r requirements.txt")
            return False

        # Check system tools
        tools = ['doxygen', 'jsdoc']
        for tool in tools:
            if not shutil.which(tool):
                print(f"❌ Missing system tool: {tool}")
                return False

        print("✅ System tools OK")
        return True

    def clean_builds(self):
        """Clean all build artifacts."""
        print("🧹 Cleaning build artifacts...")

        dirs_to_clean = [
            self.build_dir / 'html',
            self.build_dir / 'doxygen',
            self.build_dir / 'jsdoc',
            self.build_dir / 'doctrees',
            self.build_dir / 'latex'
        ]

        for dir_path in dirs_to_clean:
            if dir_path.exists():
                shutil.rmtree(dir_path)
                print(f"   Cleaned: {dir_path}")

        print("✅ Build artifacts cleaned")

    def build_sphinx(self):
        """Build Sphinx documentation."""
        print("📚 Building Sphinx documentation...")

        cmd = ['make', 'html']
        result = self.run_command(cmd)

        if result.returncode == 0:
            print("✅ Sphinx documentation built successfully")
        else:
            print("❌ Sphinx build failed")
            return False

        return True

    def build_doxygen(self):
        """Build Doxygen documentation."""
        print("⚙️ Building Doxygen documentation...")

        cmd = ['make', 'doxygen']
        result = self.run_command(cmd)

        if result.returncode == 0:
            print("✅ Doxygen documentation built successfully")
        else:
            print("❌ Doxygen build failed")
            return False

        return True

    def build_jsdoc(self):
        """Build JSDoc documentation."""
        print("🌐 Building JSDoc documentation...")

        cmd = ['make', 'jsdoc']
        result = self.run_command(cmd)

        if result.returncode == 0:
            print("✅ JSDoc documentation built successfully")
        else:
            print("❌ JSDoc build failed")
            return False

        return True

    def validate_docs(self):
        """Run documentation validation."""
        print("🔍 Validating documentation...")

        # Link check
        print("   Checking links...")
        result = self.run_command(['make', 'linkcheck'], check=False)
        if result.returncode != 0:
            print("⚠️ Link check found issues (see _build/linkcheck/output.txt)")

        # Doctest
        print("   Running doctests...")
        result = self.run_command(['make', 'doctest'], check=False)
        if result.returncode != 0:
            print("⚠️ Doctest failures found")

        print("✅ Validation completed")

    def serve_docs(self, host: str = 'localhost', port: int = 8000):
        """Serve documentation locally."""
        print(f"🚀 Serving documentation at http://{host}:{port}/")
        print("Press Ctrl+C to stop")

        try:
            cmd = ['python3', '-m', 'http.server', str(port)]
            self.run_command(cmd, cwd=self.build_dir / 'html')
        except KeyboardInterrupt:
            print("\n🛑 Server stopped")

    def build_all(self, clean: bool = False, validate: bool = False):
        """Build all documentation types."""
        start_time = time.time()

        if clean:
            self.clean_builds()

        if not self.check_dependencies():
            return False

        success = True

        # Build documentation
        if not self.build_sphinx():
            success = False
        if not self.build_doxygen():
            success = False
        if not self.build_jsdoc():
            success = False

        if validate:
            self.validate_docs()

        end_time = time.time()
        duration = end_time - start_time

        if success:
            print("🎉 All documentation built successfully!")
            print(".2f")
        else:
            print("❌ Some documentation builds failed")
            return False

        return True


def main():
    parser = argparse.ArgumentParser(
        description="Build URC 2026 documentation",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )

    parser.add_argument(
        '--clean',
        action='store_true',
        help='Clean build artifacts before building'
    )

    parser.add_argument(
        '--serve',
        action='store_true',
        help='Serve documentation locally after building'
    )

    parser.add_argument(
        '--validate',
        action='store_true',
        help='Run validation checks after building'
    )

    parser.add_argument(
        '--host',
        default=os.getenv('DOCS_HOST', 'localhost'),
        help='Host for local server (default: localhost)'
    )

    parser.add_argument(
        '--port',
        type=int,
        default=int(os.getenv('DOCS_PORT', '8000')),
        help='Port for local server (default: 8000)'
    )

    args = parser.parse_args()

    # Find docs directory
    script_dir = Path(__file__).parent
    if (script_dir / 'conf.py').exists():
        docs_dir = script_dir
    else:
        # Try to find docs directory from project root
        project_root = script_dir.parent
        docs_dir = project_root / 'docs'
        if not docs_dir.exists():
            print("❌ Cannot find docs directory")
            sys.exit(1)

    builder = DocsBuilder(docs_dir)

    # Build documentation
    success = builder.build_all(clean=args.clean, validate=args.validate)

    if success and args.serve:
        builder.serve_docs(host=args.host, port=args.port)

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
