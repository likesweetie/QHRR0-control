from __future__ import annotations

import struct
import time
from multiprocessing import shared_memory

from ..core.config import MitCommandShmConfig
from .shm_manager import (
    MIT_COMMAND_MAGIC,
    MIT_COMMAND_VERSION,
    MIT_HEADER_FMT,
    MIT_HEADER_SIZE,
    MIT_TARGET_FMT,
    MIT_TARGET_SIZE,
)
from ..core.state import MitTarget


class ShmMitCommandWriter:
    def __init__(self, config: MitCommandShmConfig, *, source_id: int = 2):
        self.config = config
        self.source_id = int(source_id)
        self._segment: shared_memory.SharedMemory | None = None
        self._seq = 0

    def close(self) -> None:
        if self._segment is not None:
            self._segment.close()
            self._segment = None

    def publish(self, targets: list[MitTarget]) -> None:
        if len(targets) != self.config.target_count:
            raise ValueError(
                f"Incomplete MIT command batch: targets={len(targets)}, expected={self.config.target_count}"
            )
        segment = self._attach()
        available_targets = (len(segment.buf) - MIT_HEADER_SIZE) // MIT_TARGET_SIZE
        if available_targets < self.config.target_count:
            raise ValueError(
                f"Incomplete MIT command SHM segment: available={available_targets}, expected={self.config.target_count}"
            )

        self._seq += 2
        odd_seq = self._seq - 1
        even_seq = self._seq
        timestamp_ns = time.time_ns()
        struct.pack_into(
            MIT_HEADER_FMT,
            segment.buf,
            0,
            MIT_COMMAND_MAGIC,
            MIT_COMMAND_VERSION,
            self.config.target_count,
            odd_seq,
            timestamp_ns,
            self.source_id,
        )
        for index, target in enumerate(targets):
            struct.pack_into(
                MIT_TARGET_FMT,
                segment.buf,
                MIT_HEADER_SIZE + index * MIT_TARGET_SIZE,
                int(target.can_id),
                float(target.position_rad),
                float(target.velocity_rad_s),
                float(target.kp),
                float(target.kd),
                float(target.torque_ff_nm),
            )
        struct.pack_into(
            MIT_HEADER_FMT,
            segment.buf,
            0,
            MIT_COMMAND_MAGIC,
            MIT_COMMAND_VERSION,
            self.config.target_count,
            even_seq,
            timestamp_ns,
            self.source_id,
        )

    def _attach(self) -> shared_memory.SharedMemory:
        if self._segment is None:
            self._segment = shared_memory.SharedMemory(name=self.config.name, create=False)
        return self._segment
