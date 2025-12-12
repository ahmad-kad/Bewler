#!/usr/bin/env python3
"""
Configuration Validation Script

Validates configuration files for production readiness and environment consistency.
"""

import os
import sys
from pathlib import Path
from typing import Any, Dict, List

import yaml


class ConfigValidator:
    """Validates configuration files for production deployment."""

    def __init__(self, config_dir: str = "config"):
        self.config_dir = Path(config_dir)
        self.errors: List[str] = []
        self.warnings: List[str] = []

    def load_yaml(self, path: Path) -> Dict[str, Any]:
        """Load YAML file safely."""
        try:
            with open(path, 'r') as f:
                return yaml.safe_load(f) or {}
        except Exception as e:
            self.errors.append(f"Failed to load {path}: {e}")
            return {}

    def validate_production_config(self) -> bool:
        """Validate production configuration."""
        prod_config = self.config_dir / "production.yaml"
        dev_config = self.config_dir / "development.yaml"

        if not prod_config.exists():
            self.errors.append("Production config file missing")
            return False

        prod_data = self.load_yaml(prod_config)
        dev_data = self.load_yaml(dev_config) if dev_config.exists() else {}

        # Check production-specific settings
        if prod_data.get('environment') != 'production':
            self.errors.append("Production config must have environment: production")

        # Hardware should not use mocks in production
        hardware = prod_data.get('hardware', {})
        if hardware.get('use_mock', True):
            self.errors.append("Production config should not use mock hardware")

        # Check for required production URLs
        required_urls = ['websocket_url', 'camera_front_url', 'camera_rear_url']
        for url_key in required_urls:
            if url_key in hardware and 'localhost' in hardware[url_key]:
                self.warnings.append(f"Production config uses localhost for {url_key}")

        return len(self.errors) == 0

    def validate_mission_config(self) -> bool:
        """Validate mission configuration."""
        mission_config = Path("config/mission_configs.yaml")

        if not mission_config.exists():
            self.errors.append("Mission config file missing")
            return False

        config = self.load_yaml(mission_config)

        # Check for environment variable placeholders
        required_env_vars = [
            'HARDWARE_USE_MOCK',
            'WEBSOCKET_URL',
            'CAN_INTERFACE',
            'CAN_ENABLED',
            'CAMERAS_ENABLED',
            'CAMERA_FRONT_URL',
            'CAMERA_REAR_URL'
        ]

        # This is just informational - we can't validate env vars at config time
        print(f"Note: Mission config uses environment variables: {', '.join(required_env_vars)}")

        return len(self.errors) == 0

    def check_environment_variables(self) -> None:
        """Check for required environment variables."""
        required_vars = [
            'ROS_DOMAIN_ID',  # ROS2 domain
            'ROS_DISCOVERY_SERVER',  # ROS2 discovery server
        ]

        for var in required_vars:
            if not os.getenv(var):
                self.warnings.append(f"Environment variable {var} not set")

    def validate_all(self) -> bool:
        """Run all validation checks."""
        print("🔍 Validating configuration for production readiness...\n")

        self.validate_production_config()
        self.validate_mission_config()
        self.check_environment_variables()

        # Report results
        if self.errors:
            print("❌ Configuration Errors:")
            for error in self.errors:
                print(f"  - {error}")

        if self.warnings:
            print("⚠️  Configuration Warnings:")
            for warning in self.warnings:
                print(f"  - {warning}")

        if not self.errors and not self.warnings:
            print("✅ All configuration checks passed!")

        return len(self.errors) == 0


def main():
    """Main validation function."""
    validator = ConfigValidator()

    if validator.validate_all():
        print("\n🎉 Configuration is production-ready!")
        sys.exit(0)
    else:
        print("\n💥 Configuration validation failed!")
        sys.exit(1)


if __name__ == "__main__":
    main()
