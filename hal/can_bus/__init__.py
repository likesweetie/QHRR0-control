"""
hal.can_bus

CAN communication package for robot control systems.

This package exposes the public CAN API:
- CAN frame abstraction
- CAN bus abstraction
- SocketCAN implementation
- CAN daemon
- CAN dispatcher
- CAN device information
"""

"""hal.can_bus.base package init"""

from frame import CANFrame
from bus import CANBus, SocketCANBus
from dispatcher import CANDispatcher
from daemon import CANDaemon
from can_types import CANFrameCallback


__all__ = [
    "CANFrame",
    "CANBus",
    "SocketCANBus",
    "CANDispatcher",
    "CANDaemon",
    "CANFrameCallback",
]