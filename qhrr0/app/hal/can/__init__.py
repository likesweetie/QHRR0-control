"""
hal.can

CAN communication package for robot control systems.

This package exposes the public CAN API:
- CAN frame abstraction
- CAN bus abstraction
- SocketCAN implementation
- CAN daemon
- CAN dispatcher
- CAN device information
"""

from .bus import CANBus, SocketCANBus
from .daemon import CANDaemon
from .dispatcher import CANDispatcher
from .frame import CANFrame


__all__ = [
    "CANFrame",
    "CANBus",
    "SocketCANBus",
    "CANDispatcher",
    "CANDaemon",
]
