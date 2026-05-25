from __future__ import annotations

import struct
import time
from multiprocessing import shared_memory

from robot_controller.core.config import MitCommandShmConfig
from robot_controller.utils.shm_manager import (
    MIT_COMMAND_MAGIC,
    MIT_COMMAND_VERSION,
    MIT_HEADER_FMT,
    MIT_HEADER_SIZE,
    MIT_TARGET_FMT,
    MIT_TARGET_SIZE,
    mit_command_shm_size,
)

from .policy_command import (
    CommandReadResult,
    CommandReadStatus,
    JointCommand,
    PolicyCommand,
)


class _ReadCollision(RuntimeError):
    pass


class ShmPolicyCommandSource:
    def __init__(self, config: MitCommandShmConfig):
        self.config = config
        self._segment: shared_memory.SharedMemory | None = None

    def close(self) -> None:
        if self._segment is not None:
            self._segment.close()
            self._segment = None

    def read_latest(self) -> CommandReadResult:
        try:
            return self._read_latest_with_retry()
        except FileNotFoundError as exc:
            return CommandReadResult(
                command=None,
                status=CommandReadStatus.SHM_UNAVAILABLE,
                reason=str(exc),
                timestamp=None,
            )
        except ValueError as exc:
            return CommandReadResult(
                command=None,
                status=CommandReadStatus.BAD_FORMAT,
                reason=str(exc),
                timestamp=None,
            )

    def is_fresh(self, result: CommandReadResult, timeout_s: float) -> bool:
        if result.command is None or result.timestamp is None:
            return False
        return (time.time() - result.timestamp) <= timeout_s

    def _read_latest_with_retry(self) -> CommandReadResult:
        deadline = time.monotonic() + 0.002
        while True:
            try:
                command = self._try_read_latest()
                if command is None:
                    return CommandReadResult(
                        command=None,
                        status=CommandReadStatus.NO_COMMAND,
                        reason="MIT command SHM has no committed command",
                        timestamp=None,
                    )
                return CommandReadResult(
                    command=command,
                    status=CommandReadStatus.AVAILABLE,
                    reason="ok",
                    timestamp=command.timestamp,
                )
            except _ReadCollision:
                if time.monotonic() >= deadline:
                    return CommandReadResult(
                        command=None,
                        status=CommandReadStatus.READ_COLLISION,
                        reason="MIT command SHM changed while reading",
                        timestamp=None,
                    )
                time.sleep(0.0002)

    def _try_read_latest(self) -> PolicyCommand | None:
        segment = self._attach()
        header = bytes(segment.buf[:MIT_HEADER_SIZE])
        if len(header) != MIT_HEADER_SIZE:
            raise ValueError("MIT command SHM header is incomplete")
        magic, version, target_count, seq_before, timestamp_ns, source_id = struct.unpack(
            MIT_HEADER_FMT,
            header,
        )
        if magic != MIT_COMMAND_MAGIC or version != MIT_COMMAND_VERSION:
            raise ValueError(f"MIT command SHM layout mismatch: magic=0x{magic:X}, version={version}")
        if target_count != self.config.target_count:
            raise ValueError(
                f"Incomplete MIT command batch: target_count={target_count}, "
                f"expected={self.config.target_count}"
            )
        if seq_before == 0:
            return None
        if seq_before % 2 != 0:
            raise _ReadCollision()

        expected_size = MIT_HEADER_SIZE + target_count * MIT_TARGET_SIZE
        if len(segment.buf) < expected_size:
            raise ValueError(
                f"MIT command SHM is too small: {len(segment.buf)} bytes, expected {expected_size}"
            )

        targets: list[JointCommand] = []
        offset = MIT_HEADER_SIZE
        for _ in range(target_count):
            can_id, position_rad, velocity_rad_s, kp, kd, torque_ff_nm = struct.unpack_from(
                MIT_TARGET_FMT,
                segment.buf,
                offset,
            )
            if can_id < 0:
                raise ValueError(f"MIT command contains invalid CAN ID {can_id}")
            targets.append(
                JointCommand(
                    can_id=int(can_id),
                    position_rad=float(position_rad),
                    velocity_rad_s=float(velocity_rad_s),
                    kp=float(kp),
                    kd=float(kd),
                    torque_ff_nm=float(torque_ff_nm),
                )
            )
            offset += MIT_TARGET_SIZE

        magic2, version2, target_count2, seq_after, timestamp_ns2, source_id2 = struct.unpack_from(
            MIT_HEADER_FMT,
            segment.buf,
            0,
        )
        if (
            magic2 != magic
            or version2 != version
            or target_count2 != target_count
            or timestamp_ns2 != timestamp_ns
            or source_id2 != source_id
            or seq_after != seq_before
            or seq_after % 2 != 0
        ):
            raise _ReadCollision()

        return PolicyCommand(
            source=f"shm:{source_id}",
            timestamp=timestamp_ns / 1_000_000_000.0,
            targets=targets,
        )

    def _attach(self) -> shared_memory.SharedMemory:
        if self._segment is None:
            self._segment = shared_memory.SharedMemory(name=self.config.name, create=False)
            expected_size = mit_command_shm_size(self.config.target_count)
            if len(self._segment.buf) != expected_size:
                self.close()
                raise ValueError(
                    f"MIT command SHM size mismatch for {self.config.name}: "
                    f"{len(self._segment.buf)} != {expected_size}"
                )
        return self._segment
