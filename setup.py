#!/usr/bin/env python3
"""
Setup script for URC 2026 Robotics Platform.

This setup.py enables editable installs and provides basic package metadata.
"""

from setuptools import find_packages, setup

# Read the README for long description
with open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()

setup(
    name="urc-machiato-2026",
    version="1.0.0",
    author="URC Machiato 2026 Team",
    author_email="",
    description="University Rover Challenge 2026 Robotics Platform",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/SJSURoboticsTeam/urc-machiato-2026",
    packages=find_packages(),
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: C++",
        "Programming Language :: JavaScript",
        "Topic :: Scientific/Engineering :: Robotics",
    ],
    python_requires=">=3.8",
    install_requires=[
        # Core documentation
        "sphinx>=5.0.0",
        "sphinx-rtd-theme>=1.2.0",

        # Multi-language support
        "breathe>=4.35.0",           # C++ documentation
        "sphinx-js>=3.1.2",          # JavaScript/TypeScript documentation
        "myst-parser>=0.18.0",       # Markdown support

        # Additional extensions
        "sphinx-copybutton>=0.5.0",  # Copy button for code blocks
        "sphinx-autodoc-typehints>=1.19.0",

        # Development and testing
        "pytest>=7.0.0",
        "pytest-cov>=4.0.0",
        "black>=22.0.0",
        "isort>=5.10.0",
        "flake8>=4.0.0",
        "mypy>=1.0.0",

        # Logging and monitoring
        "structlog>=23.1.0",

        # Data science (for analysis scripts)
        "numpy>=1.21.0",
        "pandas>=1.3.0",
        "matplotlib>=3.5.0",
        "opencv-python>=4.5.0",
    ],
    extras_require={
        "docs": [
            "sphinx>=5.0.0",
            "sphinx-rtd-theme>=1.2.0",
            "breathe>=4.35.0",
            "sphinx-js>=3.1.2",
            "myst-parser>=0.18.0",
            "sphinx-copybutton>=0.5.0",
            "sphinx-autodoc-typehints>=1.19.0",
            "sphinxcontrib-plantuml>=0.25",
            "networkx>=3.0",
            "pyyaml>=6.0",
        ],
        "dev": [
            "pytest>=7.0.0",
            "pytest-cov>=4.0.0",
            "black>=22.0.0",
            "isort>=5.10.0",
            "flake8>=4.0.0",
            "mypy>=1.0.0",
        ],
        "analysis": [
            "numpy>=1.21.0",
            "pandas>=1.3.0",
            "matplotlib>=3.5.0",
            "opencv-python>=4.5.0",
        ],
    },
    include_package_data=True,
)
