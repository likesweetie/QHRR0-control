from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import ModuleType


_ROOT = Path(__file__).resolve().parents[2]
_HAL_CAN_BUS_DIR = _ROOT / "hal" / "can_bus"


def _load_module(module_name: str, path: Path) -> ModuleType:
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Cannot load {module_name} from {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def _install_hal_can_bus_public_module(
    frame_module: ModuleType,
    bus_module: ModuleType,
    can_types_module: ModuleType,
    dispatcher_module: ModuleType,
    daemon_module: ModuleType,
) -> None:
    public_module = ModuleType("hal.can_bus")
    public_module.CANFrame = frame_module.CANFrame
    public_module.CANBus = bus_module.CANBus
    public_module.SocketCANBus = bus_module.SocketCANBus
    public_module.CANDispatcher = dispatcher_module.CANDispatcher
    public_module.CANDaemon = daemon_module.CANDaemon
    public_module.CANFrameCallback = can_types_module.CANFrameCallback
    public_module.__all__ = [
        "CANFrame",
        "CANBus",
        "SocketCANBus",
        "CANDispatcher",
        "CANDaemon",
        "CANFrameCallback",
    ]
    public_module.__path__ = [str(_HAL_CAN_BUS_DIR)]
    sys.modules["hal.can_bus"] = public_module
    sys.modules["hal.can_bus.frame"] = frame_module
    sys.modules["hal.can_bus.bus"] = bus_module
    sys.modules["hal.can_bus.can_types"] = can_types_module
    sys.modules["hal.can_bus.dispatcher"] = dispatcher_module
    sys.modules["hal.can_bus.daemon"] = daemon_module


def _load_or_get(module_name: str, path: Path, required_attr: str) -> ModuleType:
    module = sys.modules.get(module_name)
    if module is not None and hasattr(module, required_attr):
        return module
    return _load_module(module_name, path)


def _load_hal_can_modules() -> tuple[type, type, type, type, type]:
    # hal/can_bus currently uses absolute sibling imports such as
    # `from frame import CANFrame`. Load that sibling module explicitly without
    # editing HAL.
    frame_module = _load_or_get("frame", _HAL_CAN_BUS_DIR / "frame.py", "CANFrame")
    bus_module = _load_or_get("bus", _HAL_CAN_BUS_DIR / "bus.py", "SocketCANBus")
    can_types_module = _load_or_get("can_types", _HAL_CAN_BUS_DIR / "can_types.py", "CANFrameCallback")
    dispatcher_module = _load_or_get("dispatcher", _HAL_CAN_BUS_DIR / "dispatcher.py", "CANDispatcher")
    daemon_module = _load_or_get("daemon", _HAL_CAN_BUS_DIR / "daemon.py", "CANDaemon")
    _install_hal_can_bus_public_module(
        frame_module,
        bus_module,
        can_types_module,
        dispatcher_module,
        daemon_module,
    )
    return (
        frame_module.CANFrame,
        bus_module.CANBus,
        bus_module.SocketCANBus,
        dispatcher_module.CANDispatcher,
        daemon_module.CANDaemon,
    )


CANFrame, CANBus, SocketCANBus, CANDispatcher, CANDaemon = _load_hal_can_modules()

__all__ = ["CANFrame", "CANBus", "SocketCANBus", "CANDispatcher", "CANDaemon"]
