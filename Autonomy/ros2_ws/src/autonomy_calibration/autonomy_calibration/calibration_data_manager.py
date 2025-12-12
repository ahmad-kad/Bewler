#!/usr/bin/env python3
"""
Calibration Data Manager
Handles persistence, versioning, and backup of calibration data
"""

import yaml
import json
import shutil
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Optional, Any
import hashlib


class CalibrationDataManager:
    """Manages calibration data persistence with versioning and backup."""

    def __init__(self, data_directory: str, max_backups: int = 10):
        """
        Initialize data manager.

        Args:
            data_directory: Base directory for calibration data
            max_backups: Maximum number of backup versions to keep
        """
        self.data_directory = Path(data_directory)
        self.max_backups = max_backups
        self.backups_directory = self.data_directory / "backups"
        self.metadata_file = self.data_directory / "metadata.json"

        # Create directories
        self.data_directory.mkdir(parents=True, exist_ok=True)
        self.backups_directory.mkdir(parents=True, exist_ok=True)

        # Load or initialize metadata
        self.metadata = self._load_metadata()

    def _load_metadata(self) -> Dict[str, Any]:
        """Load metadata file or create default."""
        if self.metadata_file.exists():
            try:
                with open(self.metadata_file, "r") as f:
                    return json.load(f)
            except Exception:
                pass

        # Create default metadata
        return {"version": "1.0", "created": datetime.now().isoformat(), "calibrations": {}, "last_backup": None}

    def _save_metadata(self):
        """Save metadata to file."""
        with open(self.metadata_file, "w") as f:
            json.dump(self.metadata, f, indent=2)

    def _calculate_checksum(self, data: Dict) -> str:
        """Calculate checksum for data integrity."""
        data_str = json.dumps(data, sort_keys=True, default=str)
        return hashlib.sha256(data_str.encode()).hexdigest()

    def save_calibration(self, calibration_data: Dict, calibration_type: str, description: str = "") -> str:
        """
        Save calibration data with versioning.

        Args:
            calibration_data: Calibration data dictionary
            calibration_type: Type of calibration ("camera_intrinsics", "hand_eye", etc.)
            description: Optional description of the calibration

        Returns:
            Path to saved calibration file
        """
        # Add metadata
        timestamp = datetime.now()
        calibration_data["metadata"] = {
            "calibration_type": calibration_type,
            "timestamp": timestamp.isoformat(),
            "description": description,
            "version": "1.0",
            "checksum": self._calculate_checksum(calibration_data),
        }

        # Generate filename
        filename = f"{calibration_type}_{timestamp.strftime('%Y%m%d_%H%M%S')}.yaml"
        filepath = self.data_directory / filename

        # Save to YAML
        with open(filepath, "w") as f:
            yaml.dump(calibration_data, f, default_flow_style=False)

        # Update metadata
        if calibration_type not in self.metadata["calibrations"]:
            self.metadata["calibrations"][calibration_type] = []

        self.metadata["calibrations"][calibration_type].append(
            {
                "filename": filename,
                "timestamp": timestamp.isoformat(),
                "description": description,
                "checksum": calibration_data["metadata"]["checksum"],
            }
        )

        # Keep only recent calibrations (limit to max_backups)
        if len(self.metadata["calibrations"][calibration_type]) > self.max_backups:
            # Move oldest to backups
            oldest = self.metadata["calibrations"][calibration_type].pop(0)
            oldest_path = self.data_directory / oldest["filename"]
            if oldest_path.exists():
                backup_path = self.backups_directory / oldest["filename"]
                shutil.move(str(oldest_path), str(backup_path))

        self._save_metadata()

        return str(filepath)

    def load_calibration(self, calibration_type: str, version: str = "latest") -> Optional[Dict]:
        """
        Load calibration data.

        Args:
            calibration_type: Type of calibration to load
            version: "latest" or specific filename

        Returns:
            Calibration data dictionary or None if not found
        """
        if calibration_type not in self.metadata["calibrations"]:
            return None

        calibrations = self.metadata["calibrations"][calibration_type]
        if not calibrations:
            return None

        if version == "latest":
            # Get most recent
            filename = calibrations[-1]["filename"]
        else:
            # Find specific version
            matching = [c for c in calibrations if c["filename"] == version]
            if not matching:
                return None
            filename = matching[0]["filename"]

        filepath = self.data_directory / filename
        if not filepath.exists():
            # Try backups
            backup_path = self.backups_directory / filename
            if backup_path.exists():
                filepath = backup_path
            else:
                return None

        try:
            with open(filepath, "r") as f:
                data = yaml.safe_load(f)

            # Verify checksum
            if "metadata" in data and "checksum" in data["metadata"]:
                if data["metadata"]["checksum"] != self._calculate_checksum(data):
                    raise ValueError("Checksum mismatch - data may be corrupted")

            return data

        except Exception as e:
            print(f"Error loading calibration {filename}: {e}")
            return None

    def list_calibrations(self, calibration_type: Optional[str] = None) -> Dict[str, List[Dict]]:
        """
        List available calibrations.

        Args:
            calibration_type: Optional type filter

        Returns:
            Dictionary of calibration types and their versions
        """
        if calibration_type:
            return {calibration_type: self.metadata["calibrations"].get(calibration_type, [])}
        return self.metadata["calibrations"]

    def validate_calibration(self, calibration_data: Dict) -> Dict[str, Any]:
        """
        Validate calibration data integrity and quality.

        Args:
            calibration_data: Calibration data to validate

        Returns:
            Validation results dictionary
        """
        results = {"valid": True, "errors": [], "warnings": [], "quality_score": 0.0}

        try:
            # Check required fields
            if "metadata" not in calibration_data:
                results["errors"].append("Missing metadata section")
                results["valid"] = False
                return results

            metadata = calibration_data["metadata"]

            # Check calibration type
            if "calibration_type" not in metadata:
                results["errors"].append("Missing calibration type")
                results["valid"] = False

            # Check checksum
            if "checksum" in metadata:
                if metadata["checksum"] != self._calculate_checksum(calibration_data):
                    results["errors"].append("Checksum mismatch")
                    results["valid"] = False

            # Quality assessment
            if metadata.get("calibration_type") == "camera_intrinsics":
                if "reprojection_error" in calibration_data:
                    error = calibration_data["reprojection_error"]
                    if error < 0.3:
                        results["quality_score"] = 1.0  # Excellent
                    elif error < 0.5:
                        results["quality_score"] = 0.8  # Good
                    elif error < 1.0:
                        results["quality_score"] = 0.6  # Acceptable
                    else:
                        results["quality_score"] = 0.2  # Poor
                        results["warnings"].append(f"High reprojection error: {error:.4f}px")

            elif metadata.get("calibration_type") == "hand_eye":
                if "calibration_success" in calibration_data and calibration_data["calibration_success"]:
                    results["quality_score"] = 0.8  # Good if completed successfully
                else:
                    results["quality_score"] = 0.3  # Poor if failed

        except Exception as e:
            results["errors"].append(f"Validation error: {str(e)}")
            results["valid"] = False

        return results

    def backup_all_calibrations(self) -> str:
        """
        Create a complete backup of all calibration data.

        Returns:
            Path to backup archive
        """
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_name = f"calibration_backup_{timestamp}"
        backup_path = self.data_directory / f"{backup_name}.tar.gz"

        # Create backup archive
        import tarfile

        with tarfile.open(backup_path, "w:gz") as tar:
            # Add current calibrations
            for item in self.data_directory.glob("*.yaml"):
                tar.add(str(item), arcname=f"{backup_name}/{item.name}")

            # Add metadata
            tar.add(str(self.metadata_file), arcname=f"{backup_name}/metadata.json")

        # Update metadata
        self.metadata["last_backup"] = timestamp
        self._save_metadata()

        return str(backup_path)

    def cleanup_old_backups(self, max_age_days: int = 30):
        """
        Remove old backup files.

        Args:
            max_age_days: Maximum age of backups to keep
        """
        import time

        cutoff_time = time.time() - (max_age_days * 24 * 60 * 60)

        for backup_file in self.backups_directory.glob("*"):
            if backup_file.stat().st_mtime < cutoff_time:
                backup_file.unlink()

    def get_calibration_stats(self) -> Dict[str, Any]:
        """
        Get statistics about stored calibrations.

        Returns:
            Statistics dictionary
        """
        stats = {
            "total_calibrations": 0,
            "calibration_types": {},
            "newest_calibration": None,
            "oldest_calibration": None,
            "backup_count": 0,
        }

        # Count calibrations by type
        for cal_type, calibrations in self.metadata["calibrations"].items():
            stats["calibration_types"][cal_type] = len(calibrations)
            stats["total_calibrations"] += len(calibrations)

            # Find newest and oldest
            for cal in calibrations:
                if stats["newest_calibration"] is None or cal["timestamp"] > stats["newest_calibration"]:
                    stats["newest_calibration"] = cal["timestamp"]
                if stats["oldest_calibration"] is None or cal["timestamp"] < stats["oldest_calibration"]:
                    stats["oldest_calibration"] = cal["timestamp"]

        # Count backups
        stats["backup_count"] = len(list(self.backups_directory.glob("*")))

        return stats
