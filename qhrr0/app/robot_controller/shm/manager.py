from __future__ import annotations

from collections.abc import Mapping
from multiprocessing import shared_memory
from typing import Any

from ...helper.config_manage import validate_shm_config
from .types.commands import AuxCommandShm, ControlCommandShm, OperatorCommandShm
from .types.robot_state import RobotStateShm


_SHM_SEGMENTS = (
    ("mit_command", ControlCommandShm, False),
    ("aux_command", AuxCommandShm, True),
    ("operator_command", OperatorCommandShm, True),
    ("control_state", RobotStateShm, True),
    ("dashboard_state", RobotStateShm, True),
)


class ShmManager:
    def __init__(self, config: Mapping[str, Any]):
        validate_shm_config(config)
        self.config = config
        self._segments: dict[str, shared_memory.SharedMemory] = {}

    def cleanup_stale(self) -> None:
        for name in self._segment_names():
            try:
                stale = shared_memory.SharedMemory(name=name, create=False)
            except FileNotFoundError:
                continue
            stale.close()
            stale.unlink()

    def create_all(self) -> None:
        for config_key, shm_type, has_configured_size in _SHM_SEGMENTS:
            segment_config = self.config[config_key]
            name = str(segment_config["name"])
            if has_configured_size:
                segment = shm_type.create(
                    name=name,
                    size=int(segment_config["size_bytes"]),
                )
            else:
                segment = shm_type.create(name=name)
            self._segments[name] = segment.shm

    def close_all(self) -> None:
        for segment in self._segments.values():
            segment.close()
        self._segments.clear()

    def unlink_all(self) -> None:
        for name in self._segment_names():
            try:
                segment = shared_memory.SharedMemory(name=name, create=False)
            except FileNotFoundError:
                continue
            segment.close()
            segment.unlink()

    def _segment_names(self) -> tuple[str, ...]:
        return tuple(
            str(self.config[config_key]["name"])
            for config_key, _shm_type, _has_configured_size in _SHM_SEGMENTS
        )
