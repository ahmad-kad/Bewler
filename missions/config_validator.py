#!/usr/bin/env python3
"""
Configuration and Environment Validation

Validates environment variables and configuration at startup
to ensure the system is properly configured for production use.
"""

import os
import sys
from typing import Any, Dict, List, Optional

import structlog

from .exceptions import ConfigurationError, validate_required_env_var

logger = structlog.get_logger(__name__)


class ConfigValidator:
    """
    Validates configuration and environment variables at startup.
    """

    def __init__(self):
        self.errors: List[str] = []
        self.warnings: List[str] = []

    def validate_environment_variables(self) -> bool:
        """
        Validate all required environment variables are set.

        Returns:
            True if all required variables are set, False otherwise
        """
        logger.info("Validating environment variables")

        # Core ROS2 environment variables
        required_vars = [
            "ROS_DOMAIN_ID",  # Unique ROS2 domain for network isolation
            "ROS_DISCOVERY_SERVER",  # ROS2 discovery server for multi-machine setup
        ]

        # Teleoperation/WebSocket variables
        teleop_vars = [
            "WEBSOCKET_URL",  # WebSocket bridge URL
            "CAMERA_FRONT_URL",  # Front camera stream URL
            "CAMERA_REAR_URL",  # Rear camera stream URL
        ]

        # Hardware interface variables
        hardware_vars = [
            "CAN_INTERFACE",  # CAN bus interface name
            "CAN_ENABLED",  # Whether CAN is enabled
            "CAMERAS_ENABLED",  # Whether cameras are enabled
        ]

        success = True

        # Check core ROS2 variables
        for var in required_vars:
            try:
                validate_required_env_var(var)
                logger.info(f"Environment variable validated: {var}")
            except ConfigurationError as e:
                self.errors.append(str(e))
                success = False

        # Check teleoperation variables (warnings only if missing)
        for var in teleop_vars:
            if not os.getenv(var):
                self.warnings.append(f"Teleoperation variable not set: {var}")

        # Check hardware variables (warnings only if missing)
        for var in hardware_vars:
            if not os.getenv(var):
                self.warnings.append(f"Hardware variable not set: {var}")

        return success

    def validate_config_file(self, config_path: str) -> bool:
        """
        Validate configuration file exists and is readable.

        Args:
            config_path: Path to configuration file

        Returns:
            True if config file is valid, False otherwise
        """
        logger.info("Validating configuration file", config_path=config_path)

        if not os.path.exists(config_path):
            self.errors.append(f"Configuration file not found: {config_path}")
            return False

        if not os.access(config_path, os.R_OK):
            self.errors.append(f"Configuration file not readable: {config_path}")
            return False

        # Try to load and parse the config
        try:
            import yaml

            with open(config_path, "r") as f:
                config = yaml.safe_load(f)

            if not isinstance(config, dict):
                self.errors.append(
                    f"Configuration file must contain a dictionary: {config_path}"
                )
                return False

            logger.info("Configuration file validated", config_path=config_path)
            return True

        except Exception as e:
            self.errors.append(f"Failed to parse configuration file {config_path}: {e}")
            return False

    def validate_network_connectivity(self) -> bool:
        """
        Validate network connectivity to required services.

        Returns:
            True if network connectivity is OK, False otherwise
        """
        logger.info("Validating network connectivity")

        # Check ROS2 discovery server connectivity
        discovery_server = os.getenv("ROS_DISCOVERY_SERVER")
        if discovery_server:
            try:
                # Parse discovery server URL (format: host:port)
                host_port = discovery_server.split(":")
                if len(host_port) != 2:
                    self.errors.append(
                        f"Invalid ROS_DISCOVERY_SERVER format: {discovery_server}"
                    )
                    return False

                host, port = host_port
                port = int(port)

                import socket

                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5.0)  # 5 second timeout
                result = sock.connect_ex((host, port))
                sock.close()

                if result != 0:
                    self.warnings.append(
                        f"Cannot connect to ROS2 discovery server: {discovery_server}"
                    )
                else:
                    logger.info(
                        "ROS2 discovery server connectivity confirmed",
                        server=discovery_server,
                    )

            except Exception as e:
                self.warnings.append(f"Error checking ROS2 discovery server: {e}")

        return True

    def validate_all(self, config_path: Optional[str] = None) -> bool:
        """
        Run all validation checks.

        Args:
            config_path: Optional path to configuration file to validate

        Returns:
            True if all critical validations pass, False otherwise
        """
        logger.info("Starting comprehensive configuration validation")

        success = True

        # Validate environment variables
        if not self.validate_environment_variables():
            success = False

        # Validate config file if provided
        if config_path:
            if not self.validate_config_file(config_path):
                success = False

        # Validate network connectivity
        self.validate_network_connectivity()

        # Report results
        if self.errors:
            logger.error(
                "Configuration validation failed",
                error_count=len(self.errors),
                errors=self.errors,
            )

        if self.warnings:
            logger.warning(
                "Configuration validation warnings",
                warning_count=len(self.warnings),
                warnings=self.warnings,
            )

        if success:
            logger.info(
                "Configuration validation completed successfully",
                warning_count=len(self.warnings),
            )
        else:
            logger.error("Configuration validation failed")

        return success

    def get_summary(self) -> Dict[str, Any]:
        """
        Get validation summary.

        Returns:
            Dictionary with validation results
        """
        return {
            "success": len(self.errors) == 0,
            "errors": self.errors,
            "warnings": self.warnings,
            "error_count": len(self.errors),
            "warning_count": len(self.warnings),
        }


def validate_startup_configuration(config_path: Optional[str] = None) -> bool:
    """
    Validate configuration at application startup.

    This function should be called early in the application lifecycle,
    before any critical operations begin.

    Args:
        config_path: Optional path to configuration file

    Returns:
        True if startup validation passes, False otherwise

    Raises:
        SystemExit: If critical configuration errors prevent startup
    """
    validator = ConfigValidator()

    if not validator.validate_all(config_path):
        logger.critical(
            "Startup configuration validation failed - exiting", errors=validator.errors
        )
        sys.exit(1)

    return True


if __name__ == "__main__":
    # Allow running as a standalone script
    import argparse

    parser = argparse.ArgumentParser(description="Validate system configuration")
    parser.add_argument("--config", help="Path to configuration file")
    args = parser.parse_args()

    validator = ConfigValidator()
    success = validator.validate_all(args.config)

    summary = validator.get_summary()
    print(f"Validation {'PASSED' if success else 'FAILED'}")
    print(f"Errors: {summary['error_count']}")
    print(f"Warnings: {summary['warning_count']}")

    if summary["errors"]:
        print("\nErrors:")
        for error in summary["errors"]:
            print(f"  - {error}")

    if summary["warnings"]:
        print("\nWarnings:")
        for warning in summary["warnings"]:
            print(f"  - {warning}")

    sys.exit(0 if success else 1)


