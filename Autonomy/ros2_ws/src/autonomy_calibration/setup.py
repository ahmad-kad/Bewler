from setuptools import setup

package_name = "autonomy_calibration"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/calibration_manager.launch.py"]),
        ("share/" + package_name + "/config", ["config/calibration.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Robotics Team",
    maintainer_email="team@robotics2025.com",
    description="Calibration subsystem for autonomous rover operations",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "calibration_manager = autonomy_calibration.calibration_manager:main",
            "calibration_service = autonomy_calibration.calibration_service:main",
        ],
    },
)
