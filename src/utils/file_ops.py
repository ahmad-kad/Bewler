"""
File operation utilities for safe file handling.
"""

import os
from contextlib import contextmanager
from typing import Any


def ensure_directory(file_path: str):
    """Ensure the directory for a file path exists."""
    directory = os.path.dirname(file_path) if os.path.dirname(file_path) else "."
    os.makedirs(directory, exist_ok=True)


@contextmanager
def safe_file_operation(file_path: str, mode: str = "r", **kwargs):
    """Context manager for safe file operations with error handling."""
    try:
        with open(file_path, mode, **kwargs) as f:
            yield f
    except IOError as e:
        print(f"File operation failed for '{file_path}': {e}")
        raise
    except Exception as e:
        print(f"Unexpected error with file '{file_path}': {e}")
        raise


def safe_read_json(file_path: str, default_value: Any = None) -> Any:
    """Safely read a JSON file with fallback to default value."""
    try:
        with safe_file_operation(file_path, "r") as f:
            import json

            return json.load(f)
    except (IOError, ValueError):
        return default_value


def safe_write_json(file_path: str, data: Any, indent: int = 2):
    """Safely write data to a JSON file."""
    ensure_directory(file_path)
    try:
        with safe_file_operation(file_path, "w") as f:
            import json

            json.dump(data, f, indent=indent)
    except Exception as e:
        print(f"Failed to write JSON to '{file_path}': {e}")
        raise


def safe_read_yaml(file_path: str, default_value: Any = None) -> Any:
    """Safely read a YAML file with fallback to default value."""
    try:
        with safe_file_operation(file_path, "r") as f:
            import yaml

            return yaml.safe_load(f) or default_value
    except ImportError:
        print("PyYAML not available, cannot read YAML files")
        return default_value
    except (IOError, yaml.YAMLError):
        return default_value


def safe_write_yaml(file_path: str, data: Any):
    """Safely write data to a YAML file."""
    ensure_directory(file_path)
    try:
        with safe_file_operation(file_path, "w") as f:
            import yaml

            yaml.dump(data, f, default_flow_style=False, indent=2)
    except ImportError:
        print("PyYAML not available, cannot write YAML files")
        raise
    except Exception as e:
        print(f"Failed to write YAML to '{file_path}': {e}")
        raise


def get_file_size_mb(file_path: str) -> float:
    """Get file size in megabytes."""
    try:
        return os.path.getsize(file_path) / (1024 * 1024)
    except OSError:
        return 0.0


def cleanup_temp_files(directory: str = ".", pattern: str = "*.tmp"):
    """Clean up temporary files matching a pattern."""
    import glob

    temp_files = glob.glob(os.path.join(directory, pattern))
    for temp_file in temp_files:
        try:
            os.unlink(temp_file)
            print(f"Cleaned up: {temp_file}")
        except OSError:
            pass
