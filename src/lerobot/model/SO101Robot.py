"""Compatibility shim for SO101Robot.

Some parts of the codebase import `lerobot.model.SO101Robot` while the
actual implementation lives under `lerobot.robots.xlerobot.src.model`.
This module re-exports the expected symbols so running from source works
without installing the package.
"""
try:
    from lerobot.robots.xlerobot.src.model.SO101Robot import (
        create_real_robot,
        SO101Kinematics,
    )
except Exception as e:  # pragma: no cover - fallback for import errors
    raise

__all__ = ["create_real_robot", "SO101Kinematics"]
