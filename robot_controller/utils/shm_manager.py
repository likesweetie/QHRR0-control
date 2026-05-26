from __future__ import annotations

import struct
from multiprocessing import shared_memory

from ..core.config import ShmConfig
from ..core.robot_state_shm import (
    ROBOT_STATE_HEADER_FMT,
    ROBOT_STATE_MAGIC,
    ROBOT_STATE_VERSION,
)
from robot_controller.shm.control_command import ControlCommandShm
from robot_controller.shm.operator_command import OperatorCommandShm
from robot_controller.shm.robot_state import RobotStateShm


MIT_COMMAND_MAGIC = 0x4D495443
MIT_COMMAND_VERSION = 1
MIT_HEADER_FMT = "<IIIQQI"
MIT_TARGET_FMT = "<iddddd"
MIT_HEADER_SIZE = struct.calcsize(MIT_HEADER_FMT)
MIT_TARGET_SIZE = struct.calcsize(MIT_TARGET_FMT)


def mit_command_shm_size(target_count: int) -> int:
    if target_count <= 0:
        raise ValueError(f"MIT command target_count must be > 0: {target_count}")
    return MIT_HEADER_SIZE + int(target_count) * MIT_TARGET_SIZE


class ShmManager:
    def __init__(self, config: ShmConfig):
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
        control_command = ControlCommandShm.create(self.config.mit_command.name)
        self._segments[self.config.mit_command.name] = control_command.shm

        control_state = RobotStateShm.create(
            name=self.config.control_state.name,
            size=int(self.config.control_state.size_bytes),
        )
        self._segments[self.config.control_state.name] = control_state.shm

        aux_command = shared_memory.SharedMemory(
            name=self.config.aux_command.name,
            create=True,
            size=int(self.config.aux_command.size_bytes),
        )
        self._segments[self.config.aux_command.name] = aux_command
        self._init_robot_state_segment(aux_command)

        operator_command = OperatorCommandShm.create(
            name=self.config.operator_command.name,
            size=int(self.config.operator_command.size_bytes),
        )
        self._segments[self.config.operator_command.name] = operator_command.shm

        dashboard_state = RobotStateShm.create(
            name=self.config.dashboard_state.name,
            size=int(self.config.dashboard_state.size_bytes),
        )
        self._segments[self.config.dashboard_state.name] = dashboard_state.shm

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
        return (
            self.config.mit_command.name,
            self.config.aux_command.name,
            self.config.operator_command.name,
            self.config.control_state.name,
            self.config.dashboard_state.name,
        )

    def _init_mit_command_segment(self, segment: shared_memory.SharedMemory) -> None:
        segment.buf[:] = b"\x00" * len(segment.buf)
        struct.pack_into(
            MIT_HEADER_FMT,
            segment.buf,
            0,
            MIT_COMMAND_MAGIC,
            MIT_COMMAND_VERSION,
            int(self.config.mit_command.target_count),
            0,
            0,
            0,
        )

    def _init_robot_state_segment(self, segment: shared_memory.SharedMemory) -> None:
        segment.buf[:] = b"\x00" * len(segment.buf)
        struct.pack_into(
            ROBOT_STATE_HEADER_FMT,
            segment.buf,
            0,
            ROBOT_STATE_MAGIC,
            ROBOT_STATE_VERSION,
            0,
            0,
            0,
        )
