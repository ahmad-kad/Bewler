#!/usr/bin/env python3
"""
Production Health Check Script

Performs comprehensive checks to ensure the system is ready for production deployment.
"""

import os
import subprocess
import sys
from pathlib import Path
from typing import List


class ProductionHealthCheck:
    """Comprehensive production readiness checker."""

    def __init__(self):
        self.checks = []
        self.errors = []
        self.warnings = []

    def add_check(self, name: str, check_func, *args, **kwargs):
        """Add a check to the test suite."""
        self.checks.append((name, check_func, args, kwargs))

    def log_error(self, message: str):
        """Log an error."""
        self.errors.append(message)
        print(f"❌ {message}")

    def log_warning(self, message: str):
        """Log a warning."""
        self.warnings.append(message)
        print(f"⚠️  {message}")

    def log_success(self, message: str):
        """Log a success."""
        print(f"✅ {message}")

    def check_file_exists(self, file_path: str, description: str) -> bool:
        """Check if a file exists."""
        if os.path.exists(file_path):
            self.log_success(f"{description} found")
            return True
        else:
            self.log_error(f"{description} missing: {file_path}")
            return False

    def check_command_available(self, command: str, description: str) -> bool:
        """Check if a command is available."""
        try:
            subprocess.run([command, "--version"], capture_output=True, check=True)
            self.log_success(f"{description} available")
            return True
        except (subprocess.CalledProcessError, FileNotFoundError):
            self.log_error(f"{description} not available: {command}")
            return False

    def check_python_imports(self, modules: List[str]) -> bool:
        """Check if Python modules can be imported."""
        success = True
        for module in modules:
            try:
                __import__(module)
                self.log_success(f"Python module available: {module}")
            except ImportError:
                self.log_error(f"Python module missing: {module}")
                success = False
        return success

    def check_config_validation(self) -> bool:
        """Check if configuration validation passes."""
        try:
            result = subprocess.run([
                sys.executable, "scripts/validate_config.py"
            ], capture_output=True, text=True, cwd=Path(__file__).parent.parent)

            if result.returncode == 0:
                self.log_success("Configuration validation passed")
                return True
            else:
                self.log_error("Configuration validation failed")
                print(result.stdout)
                print(result.stderr)
                return False
        except Exception as e:
            self.log_error(f"Configuration validation error: {e}")
            return False

    def check_docker_available(self) -> bool:
        """Check if Docker is available."""
        return self.check_command_available("docker", "Docker")

    def check_git_status(self) -> bool:
        """Check Git repository status."""
        try:
            # Check if we're in a git repo
            result = subprocess.run(["git", "status"], capture_output=True, text=True)
            if result.returncode == 0:
                self.log_success("Git repository healthy")
                return True
            else:
                self.log_error("Git repository issues")
                return False
        except FileNotFoundError:
            self.log_warning("Git not available")
            return False

    def run_all_checks(self) -> bool:
        """Run all health checks."""
        print("🏥 Production Health Check\n")

        # File existence checks
        self.check_file_exists("pyproject.toml", "Project configuration")
        self.check_file_exists("config/production.yaml", "Production config")
        self.check_file_exists("config/development.yaml", "Development config")
        self.check_file_exists("DEPLOYMENT.md", "Deployment documentation")
        self.check_file_exists("README.md", "Project README")

        # Tool availability checks
        self.check_command_available("python3", "Python 3")
        self.check_docker_available()

        # Python module checks (core dependencies)
        core_modules = ["yaml", "pathlib", "subprocess"]
        self.check_python_imports(core_modules)

        # Configuration validation
        self.check_config_validation()

        # Git status
        self.check_git_status()

        # Summary
        print("\n📊 Health Check Summary:")
        print(f"  Errors: {len(self.errors)}")
        print(f"  Warnings: {len(self.warnings)}")

        if self.errors:
            print("\n💥 Production readiness failed!")
            return False
        else:
            print("\n🎉 System passed health checks!")
            if self.warnings:
                print("Address warnings for optimal production readiness.")
            return True


def main():
    """Main health check function."""
    checker = ProductionHealthCheck()
    success = checker.run_all_checks()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
