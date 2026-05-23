"""
HAL.CAN

CAN communication package for robot control systems.

This package exposes the public CAN API:
- CAN frame abstraction
- CAN bus abstraction
- SocketCAN implementation
- CAN daemon
- CAN dispatcher
- CAN device information
"""

from .base import (
    CANFrame,
    CANBus,
    SocketCANBus,
    CANDispatcher,
    CANDaemon,
    CANFrameCallback
)

__all__ = [
    "CANFrame",
    "CANBus",
    "SocketCANBus",
    "CANDispatcher",
    "CANDaemon",
    "CANFrameCallback"
]