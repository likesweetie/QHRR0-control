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
from ..core.state import MitCommandBatch, MitTarget


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
        deadline = time.monotonic() + 0.002
        while True:
            try:
                return self._try_read_latest_batch(segment)
            except _MitCommandReadCollision:
                if time.monotonic() >= deadline:
                    return None
                time.sleep(0.0002)

    def _try_read_latest_batch(self, segment: shared_memory.SharedMemory) -> MitCommandBatch | None:

        first = self._read_header(segment)
        if first is None:
            raise ValueError("MIT command SHM header is incomplete")
        magic, version, target_count, seq_before, timestamp_ns, source_id = first
        if magic != MIT_COMMAND_MAGIC or version != MIT_COMMAND_VERSION:
            raise ValueError(
                f"MIT command SHM layout mismatch: magic=0x{magic:X}, version={version}"
            )
        if seq_before == 0 or seq_before % 2 != 0:
            raise _MitCommandReadCollision()

        targets: list[MitTarget] = []
        if len(segment.buf) < MIT_HEADER_SIZE:
            raise ValueError("MIT command SHM segment is smaller than the header")
        available_targets = (len(segment.buf) - MIT_HEADER_SIZE) // MIT_TARGET_SIZE
        if int(target_count) != self.config.target_count:
            raise ValueError(
                "Incomplete MIT command batch: "
                f"target_count={target_count}, expected={self.config.target_count}"
            )
        if available_targets < self.config.target_count:
            raise ValueError(
                "Incomplete MIT command SHM segment: "
                f"available_targets={available_targets}, expected={self.config.target_count}"
            )
        for index in range(self.config.target_count):
            offset = MIT_HEADER_SIZE + index * MIT_TARGET_SIZE
            can_id, position, velocity, kp, kd, torque = struct.unpack_from(
                MIT_TARGET_FMT,
                segment.buf,
                offset,
            )
            if can_id < 0:
                raise ValueError(f"Invalid MIT target CAN ID at index {index}: {can_id}")
            targets.append(
                MitTarget(
                    can_id=int(can_id),
                    position_rad=float(position),
                    velocity_rad_s=float(velocity),
                    kp=float(kp),
                    kd=float(kd),
                    torque_ff_nm=float(torque),
                )
            )

        second = self._read_header(segment)
        if second is None or second[3] != seq_before or second[3] % 2 != 0:
            raise _MitCommandReadCollision()

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


class _MitCommandReadCollision(Exception):
    pass


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
