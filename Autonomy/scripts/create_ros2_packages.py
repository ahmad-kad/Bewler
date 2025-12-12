#!/usr/bin/env python3
"""
🚀 ROS 2 Package Creator

Quickly create ROS 2 packages for all subsystems to unblock development.
"""

import os
from pathlib import Path

PACKAGE_TEMPLATE = {
    "package.xml": """<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{package_name}</name>
  <version>0.0.1</version>
  <description>URC 2026 {description}</description>
  <maintainer email="team@urc2026.edu">URC 2026 Team</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>sensor_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
""",
    "setup.py": """from setuptools import setup

package_name = '{package_name}'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='URC 2026 Team',
    maintainer_email='team@urc2026.edu',
    description='URC 2026 {description}',
    license='MIT',
    tests_require=['pytest'],
    entry_points={{
        'console_scripts': [
            '{package_name}_node = {package_name}.{package_name}_node:main',
        ],
    }},
)
""",
    "setup.cfg": """[develop]
script_dir=$base/lib/{package_name}
[install]
install_scripts=$base/lib/{package_name}
""",
    "resource/package_name": "",
    "src/__init__.py": "",
    "src/{package_name}_node.py": '''#!/usr/bin/env python3
"""
🚀 {description} Node

URC 2026 {description} subsystem implementation.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class {class_name}Node(Node):
    """{description} node implementation."""

    def __init__(self):
        super().__init__('{package_name}_node')
        self.get_logger().info('{description} node started')

        # TODO: Add publishers, subscribers, services
        self.status_publisher = self.create_publisher(
            String,
            '{package_name}/status',
            10
        )

        # Publish status every second
        self.timer = self.create_timer(1.0, self.publish_status)

    def publish_status(self):
        """Publish current status."""
        msg = String()
        msg.data = '{package_name} operational'
        self.status_publisher.publish(msg)
        self.get_logger().debug('Published status')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = {class_name}Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
''',
}

SUBSYSTEMS = {
    "navigation": {"description": "Navigation Subsystem", "class_name": "Navigation"},
    "slam": {"description": "SLAM Subsystem", "class_name": "Slam"},
    "computer_vision": {"description": "Computer Vision Subsystem", "class_name": "ComputerVision"},
    "autonomous_typing": {"description": "Autonomous Typing Subsystem", "class_name": "AutonomousTyping"},
    "state_management": {"description": "State Management Subsystem", "class_name": "StateManagement"},
    "led_status": {"description": "LED Status Subsystem", "class_name": "LedStatus"},
}


def create_package(package_name: str, info: dict):
    """Create a ROS 2 package for the given subsystem."""
    package_dir = Path(f"Autonomy/code/{package_name}")

    print(f"📦 Creating ROS 2 package: {package_name}")

    # Create directory structure
    (package_dir / "resource").mkdir(parents=True, exist_ok=True)
    (package_dir / "src").mkdir(exist_ok=True)

    # Create files from templates
    for filename, template in PACKAGE_TEMPLATE.items():
        if filename == "resource/package_name":
            # Special case for resource file
            file_path = package_dir / "resource" / package_name
            file_path.write_text("")
        elif filename == "src/__init__.py":
            (package_dir / filename).write_text("")
        else:
            # Format template
            content = template.format(
                package_name=package_name, description=info["description"], class_name=info["class_name"]
            )

            if "src/" in filename:
                # Handle src files
                actual_filename = filename.replace("src/", "").replace("{package_name}", package_name)
                file_path = package_dir / "src" / actual_filename
            else:
                file_path = package_dir / filename

            file_path.write_text(content)

    print(f"✅ Created {package_name} package")


def test_package_compilation():
    """Test that all packages compile."""
    print("🔨 Testing package compilation...")

    os.chdir("Autonomy")

    # Run colcon build
    import subprocess

    result = subprocess.run(
        [
            "colcon",
            "build",
            "--packages-select",
            "navigation",
            "slam",
            "computer_vision",
            "autonomous_typing",
            "state_management",
            "led_status",
        ],
        capture_output=True,
        text=True,
    )

    if result.returncode == 0:
        print("✅ All packages compiled successfully!")
        return True
    else:
        print("❌ Compilation failed:")
        print(result.stdout)
        print(result.stderr)
        return False


def main():
    """Create all ROS 2 packages."""
    print("🚀 Creating ROS 2 packages for all subsystems...")
    print("=" * 50)

    for package_name, info in SUBSYSTEMS.items():
        create_package(package_name, info)

    print("\n🔨 Testing compilation...")
    success = test_package_compilation()

    if success:
        print("\n🎉 SUCCESS: All ROS 2 packages created and compiled!")
        print("\nNext steps:")
        print("1. Push changes to trigger CI/CD validation")
        print("2. Start implementing basic functionality")
        print("3. Run: python Autonomy/scripts/daily_velocity_check.py")
    else:
        print("\n⚠️  Some packages failed compilation - check errors above")


if __name__ == "__main__":
    main()
