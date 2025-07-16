from setuptools import setup, find_packages

setup(
    name="pid-autotune",
    version="1.0.0",
    description="Automatic PID tuning and filtering optimization for ArduPilot",
    author="ArduPilot Tuning Tool",
    packages=find_packages(),
    install_requires=[
        "numpy>=1.21.0",
        "pandas>=1.3.0",
        "scipy>=1.7.0",
        "matplotlib>=3.4.0",
        "pymavlink>=2.4.0",
        "click>=8.0.0",
    ],
    entry_points={
        "console_scripts": [
            "pid-autotune=pid_autotune.cli:main",
        ],
    },
    python_requires=">=3.7",
)