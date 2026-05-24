from __future__ import annotations

import struct
import time
from multiprocessing import shared_memory

from .config import MitCommandShmConfig
from .shm_manager import (
    MIT_COMMAND_MAGIC,
    MIT_COMMAND_VERSION,
    MIT_HEADER_FMT,
    MIT_HEADER_SIZE,
    MIT_TARGET_FMT,
    MIT_TARGET_SIZE,
)
from .state import MitCommandBatch, MitTarget


class ShmMitCommandRouter:
    def __init__(self, config: MitCommandShmConfig):
        self.config = config
        self._segment: shared_memory.SharedMemory | None = None

    def close(self) -> None:
        if self._segment is not None:
            self._segment.close()
            self._segment = None

    def read_latest_batch(self) -> MitCommandBatch | None:
        segment = self._attach()
        if segment is None:
            return None

        first = self._read_header(segment)
        if first is None:
            return None
        magic, version, motor_count, seq_before, timestamp_ns, source_id = first
        if magic != MIT_COMMAND_MAGIC or version != MIT_COMMAND_VERSION:
            return None
        if seq_before == 0 or seq_before % 2 != 0:
            return None

        targets: list[MitTarget] = []
        available_targets = max(0, (len(segment.buf) - MIT_HEADER_SIZE) // MIT_TARGET_SIZE)
        count = min(int(motor_count), self.config.motor_count, available_targets)
        for index in range(count):
            offset = MIT_HEADER_SIZE + index * MIT_TARGET_SIZE
            motor_id, position, velocity, kp, kd, torque = struct.unpack_from(
                MIT_TARGET_FMT,
                segment.buf,
                offset,
            )
            if motor_id < 0:
                continue
            targets.append(
                MitTarget(
                    motor_id=int(motor_id),
                    position_rad=float(position),
                    velocity_rad_s=float(velocity),
                    kp=float(kp),
                    kd=float(kd),
                    torque_ff_nm=float(torque),
                )
            )

        second = self._read_header(segment)
        if second is None or second[3] != seq_before or second[3] % 2 != 0:
            return None

        timestamp = float(timestamp_ns) / 1_000_000_000.0
        return MitCommandBatch(
            source=f"shm:{source_id}",
            timestamp=timestamp,
            targets=targets,
        )

    def is_fresh(self, batch: MitCommandBatch, timeout_s: float) -> bool:
        if batch.timestamp <= 0.0:
            return False
        return (time.time() - batch.timestamp) <= timeout_s

    def _attach(self) -> shared_memory.SharedMemory | None:
        if self._segment is not None:
            return self._segment
        try:
            self._segment = shared_memory.SharedMemory(
                name=self.config.name,
                create=False,
            )
        except FileNotFoundError:
            return None
        return self._segment

    @staticmethod
    def _read_header(segment: shared_memory.SharedMemory) -> tuple[int, int, int, int, int, int] | None:
        if len(segment.buf) < MIT_HEADER_SIZE:
            return None
        return struct.unpack_from(MIT_HEADER_FMT, segment.buf, 0)
