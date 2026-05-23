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

from base.frame import CANFrame
from base.bus import CANBus, SocketCANBus
from base.dispatcher import CANDispatcher
from base.daemon import CANDaemon
from base.can_types import CANFrameCallback


__all__ = [
    "CANFrame",
    "CANBus",
    "SocketCANBus",
    "CANDispatcher",
    "CANDaemon",
    "CANFrameCallback",
]