from __future__ import annotations

import struct
from multiprocessing import shared_memory

from .config import ShmConfig


MIT_COMMAND_MAGIC = 0x4D495443
MIT_COMMAND_VERSION = 1
MIT_HEADER_FMT = "<IIIQQI"
MIT_TARGET_FMT = "<iddddd"
MIT_HEADER_SIZE = struct.calcsize(MIT_HEADER_FMT)
MIT_TARGET_SIZE = struct.calcsize(MIT_TARGET_FMT)


def mit_command_shm_size(motor_count: int) -> int:
    return MIT_HEADER_SIZE + max(0, int(motor_count)) * MIT_TARGET_SIZE


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
        mit_size = mit_command_shm_size(self.config.mit_command.motor_count)
        mit = shared_memory.SharedMemory(
            name=self.config.mit_command.name,
            create=True,
            size=mit_size,
        )
        self._segments[self.config.mit_command.name] = mit
        self._init_mit_command_segment(mit)

        if self.config.robot_state.enabled:
            robot_state = shared_memory.SharedMemory(
                name=self.config.robot_state.name,
                create=True,
                size=4096,
            )
            robot_state.buf[:] = b"\x00" * len(robot_state.buf)
            self._segments[self.config.robot_state.name] = robot_state

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
        names = [self.config.mit_command.name]
        if self.config.robot_state.enabled:
            names.append(self.config.robot_state.name)
        return tuple(names)

    def _init_mit_command_segment(self, segment: shared_memory.SharedMemory) -> None:
        segment.buf[:] = b"\x00" * len(segment.buf)
        struct.pack_into(
            MIT_HEADER_FMT,
            segment.buf,
            0,
            MIT_COMMAND_MAGIC,
            MIT_COMMAND_VERSION,
            int(self.config.mit_command.motor_count),
            0,
            0,
            0,
        )
