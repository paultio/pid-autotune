"""
PID Auto-Tuning Tool for ArduPilot
Automatically analyzes dataflash logs and optimizes PID and filtering parameters
"""

from .main import PIDAutoTuner

__version__ = "1.0.0"
__author__ = "ArduPilot Tuning Tool"
__all__ = ['PIDAutoTuner']