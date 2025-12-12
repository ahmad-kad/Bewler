#!/usr/bin/env python3
"""
Infrastructure Validation Script for URC 2026 Autonomy Project

This script validates that the core ROS2 infrastructure is properly set up
and ready for development.
"""

import sys
from pathlib import Path


class InfrastructureValidator:
    """Validates ROS2 project infrastructure"""

    def __init__(self, project_root: str):
        self.project_root = Path(project_root)
        self.autonomy_dir = self.project_root / "Autonomy"
        self.ros2_ws = self.autonomy_dir / "ros2_ws"
        self.src_dir = self.ros2_ws / "src"
        self.results = {}

    def validate_workspace_structure(self) -> bool:
        """Validate ROS2 workspace structure"""
        print("🔍 Validating ROS2 workspace structure...")

        required_paths = [
            self.ros2_ws,
            self.src_dir,
        ]

        for path in required_paths:
            if not path.exists():
                self.results["workspace"] = f"❌ Missing: {path}"
                return False

        # Check for package symlinks
        expected_packages = [
            "autonomy_state_management",
            "autonomy_navigation",
            "autonomy_slam",
            "autonomy_computer_vision",
            "autonomy_autonomous_typing",
            "autonomy_led_status",
        ]

        missing_packages = []
        for pkg in expected_packages:
            pkg_path = self.src_dir / pkg
            if not pkg_path.exists():
                missing_packages.append(pkg)

        if missing_packages:
            self.results["workspace"] = f"❌ Missing packages: {missing_packages}"
            return False

        self.results["workspace"] = "✅ ROS2 workspace structure is valid"
        return True

    def validate_package_structure(self) -> bool:
        """Validate ROS2 package structure"""
        print("🔍 Validating ROS2 package structures...")

        packages_valid = True
        package_issues = []

        for pkg_dir in self.src_dir.glob("autonomy_*"):
            if not pkg_dir.is_dir():
                continue

            pkg_name = pkg_dir.name

            # Check if this is an interface package (has msg/, srv/, or action/ directories)
            has_interfaces = any((pkg_dir / d).exists() for d in ["msg", "srv", "action"])

            if has_interfaces:
                # Interface packages should be CMake-based
                required_files = ["package.xml", "CMakeLists.txt"]
            else:
                # Regular packages should have either Python or CMake setup
                has_python = (pkg_dir / "setup.py").exists() and (pkg_dir / "setup.cfg").exists()
                has_cmake = (pkg_dir / "CMakeLists.txt").exists()

                if not (has_python or has_cmake):
                    package_issues.append(
                        f"{pkg_name}: missing both Python (setup.py/setup.cfg) and CMake (CMakeLists.txt) files"
                    )
                    packages_valid = False
                    continue

                required_files = ["package.xml"]

            missing_files = []
            for file in required_files:
                if not (pkg_dir / file).exists():
                    missing_files.append(file)

            if missing_files:
                package_issues.append(f"{pkg_name}: missing {missing_files}")
                packages_valid = False

            # Check for Python package structure (only for Python packages)
            if not has_interfaces and has_python:
                py_pkg_dir = pkg_dir / pkg_name
                if not py_pkg_dir.exists():
                    package_issues.append(f"{pkg_name}: missing Python package directory")
                    packages_valid = False
                elif not (py_pkg_dir / "__init__.py").exists():
                    package_issues.append(f"{pkg_name}: missing __init__.py")
                    packages_valid = False

            # Check for resource marker
            resource_dir = pkg_dir / "resource"
            if resource_dir.exists():
                resource_file = resource_dir / pkg_name
                if not resource_file.exists():
                    package_issues.append(f"{pkg_name}: missing resource marker")
                    packages_valid = False

        if package_issues:
            self.results["packages"] = f"❌ Package issues: {package_issues}"
            return False

        self.results["packages"] = "✅ All ROS2 packages are properly structured"
        return True

    def validate_ci_cd_setup(self) -> bool:
        """Validate CI/CD setup"""
        print("🔍 Validating CI/CD setup...")

        workflow_file = self.project_root / ".github" / "workflows" / "ci.yml"
        if not workflow_file.exists():
            self.results["ci_cd"] = "❌ Missing GitHub Actions workflow"
            return False

        self.results["ci_cd"] = "✅ CI/CD pipeline configured"
        return True

    def validate_testing_framework(self) -> bool:
        """Validate testing framework"""
        print("🔍 Validating testing framework...")

        test_files = [self.autonomy_dir / "tests" / "integration_test.py"]

        missing_tests = []
        for test_file in test_files:
            if not test_file.exists():
                missing_tests.append(str(test_file))

        if missing_tests:
            self.results["testing"] = f"❌ Missing test files: {missing_tests}"
            return False

        self.results["testing"] = "✅ Testing framework configured"
        return True

    def validate_development_tools(self) -> bool:
        """Validate development tools"""
        print("🔍 Validating development tools...")

        tools_present = True
        tool_issues = []

        # Check Docker setup
        docker_compose = self.autonomy_dir / "development" / "docker" / "docker-compose.yml"
        if not docker_compose.exists():
            tool_issues.append("Missing docker-compose.yml")
            tools_present = False

        # Check scripts
        scripts = [
            self.autonomy_dir / "development" / "docker" / "scripts" / "docker-dev.sh",
            self.autonomy_dir / "scripts" / "daily_velocity_check.py",
            self.autonomy_dir / "scripts" / "velocity_tools.py",
        ]

        for script in scripts:
            if not script.exists():
                tool_issues.append(f"Missing script: {script.name}")
                tools_present = False

        if tool_issues:
            self.results["tools"] = f"❌ Tool issues: {tool_issues}"
            return False

        self.results["tools"] = "✅ Development tools configured"
        return True

    def run_validation(self) -> bool:
        """Run all validations"""
        print("🚀 Starting Infrastructure Validation for URC 2026 Autonomy Project")
        print("=" * 70)

        validations = [
            self.validate_workspace_structure,
            self.validate_package_structure,
            self.validate_ci_cd_setup,
            self.validate_testing_framework,
            self.validate_development_tools,
        ]

        all_passed = True
        for validation in validations:
            try:
                passed = validation()
                if not passed:
                    all_passed = False
                print()
            except Exception as e:
                print(f"❌ Validation error: {e}")
                all_passed = False

        print("=" * 70)
        print("📊 VALIDATION RESULTS:")
        print("=" * 70)

        for component, result in self.results.items():
            print(f"{result}")

        print("=" * 70)

        if all_passed:
            print("🎉 INFRASTRUCTURE VALIDATION: PASSED")
            print("✅ The ROS2 project infrastructure is ready for development!")
            print("\n📋 Next Steps:")
            print("1. Start Docker environment: cd development/docker && ./scripts/docker-dev.sh build")
            print("2. Build ROS2 packages: cd ros2_ws && colcon build")
            print("3. Run tests: python3 tests/integration_test.py")
            print("4. Begin development following the 40-day sprint plan")
        else:
            print("❌ INFRASTRUCTURE VALIDATION: FAILED")
            print("🔧 Please fix the issues above before proceeding with development.")

        return all_passed


def main():
    """Main entry point"""
    project_root = Path(__file__).parent.parent
    validator = InfrastructureValidator(project_root)
    success = validator.run_validation()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
