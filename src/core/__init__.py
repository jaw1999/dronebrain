"""
Core system components for DroneBrain.

This package contains the central pipeline manager and shared state management.
"""

from .state import SystemState
from .pipeline_manager import PipelineManager

__all__ = ["SystemState", "PipelineManager"]
