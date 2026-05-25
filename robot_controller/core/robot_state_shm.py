from __future__ import annotations

import json
import struct
import time
from multiprocessing import shared_memory
from typing import Any


ROBOT_STATE_MAGIC = 0x52535453
ROBOT_STATE_VERSION = 1
ROBOT_STATE_HEADER_FMT = "<IIQQI"
ROBOT_STATE_HEADER_SIZE = struct.calcsize(ROBOT_STATE_HEADER_FMT)
ROBOT_STATE_SEQ_OFFSET = struct.calcsize("<II")
ROBOT_STATE_TIMESTAMP_OFFSET = struct.calcsize("<IIQ")
ROBOT_STATE_PAYLOAD_LEN_OFFSET = struct.calcsize("<IIQQ")


class RobotStateShmWriter:
    def __init__(self, name: str, size_bytes: int) -> None:
        self.name = name
        self.size_bytes = int(size_bytes)
        self._segment: shared_memory.SharedMemory | None = None
        self._seq = 0

    def close(self) -> None:
        if self._segment is not None:
            self._segment.close()
            self._segment = None

    def publish(self, snapshot: dict[str, Any]) -> None:
        segment = self._attach()
        payload = json.dumps(snapshot, separators=(",", ":"), allow_nan=False).encode("utf-8")
        capacity = len(segment.buf) - ROBOT_STATE_HEADER_SIZE
        if len(payload) > capacity:
            raise RuntimeError(
                f"Robot state payload is too large for SHM: {len(payload)}/{capacity} bytes"
            )

        self._seq += 2
        odd_seq = self._seq - 1
        even_seq = self._seq
        timestamp_ns = time.time_ns()

        struct.pack_into("<II", segment.buf, 0, ROBOT_STATE_MAGIC, ROBOT_STATE_VERSION)
        struct.pack_into("<Q", segment.buf, ROBOT_STATE_SEQ_OFFSET, odd_seq)
        segment.buf[ROBOT_STATE_HEADER_SIZE : ROBOT_STATE_HEADER_SIZE + len(payload)] = payload
        struct.pack_into("<Q", segment.buf, ROBOT_STATE_TIMESTAMP_OFFSET, timestamp_ns)
        struct.pack_into("<I", segment.buf, ROBOT_STATE_PAYLOAD_LEN_OFFSET, len(payload))
        # Commit the seqlock last. Readers treat odd seq as in-progress.
        struct.pack_into("<Q", segment.buf, ROBOT_STATE_SEQ_OFFSET, even_seq)

    def _attach(self) -> shared_memory.SharedMemory:
        if self._segment is None:
            self._segment = shared_memory.SharedMemory(name=self.name, create=False)
            if len(self._segment.buf) != self.size_bytes:
                self.close()
                raise RuntimeError(f"Robot state SHM size mismatch for {self.name}")
        return self._segment


class RobotStateShmReader:
    def __init__(self, name: str) -> None:
        self.name = name
        self._segment: shared_memory.SharedMemory | None = None

    def close(self) -> None:
        if self._segment is not None:
            self._segment.close()
            self._segment = None

    def read_latest(self) -> dict[str, Any] | None:
        segment = self._attach()
        first = self._read_header(segment)
        if first is None:
            return None
        magic, version, seq_before, _timestamp_ns, payload_len = first
        if magic != ROBOT_STATE_MAGIC or version != ROBOT_STATE_VERSION:
            raise RuntimeError(
                f"Robot state SHM layout mismatch: magic=0x{magic:X}, version={version}"
            )
        if seq_before == 0 or seq_before % 2 != 0:
            return None

        capacity = len(segment.buf) - ROBOT_STATE_HEADER_SIZE
        if payload_len <= 0 or payload_len > capacity:
            raise RuntimeError(f"Invalid robot state payload length: {payload_len}/{capacity}")

        payload = bytes(segment.buf[ROBOT_STATE_HEADER_SIZE : ROBOT_STATE_HEADER_SIZE + payload_len])
        second = self._read_header(segment)
        if second is None or second[2] != seq_before or second[2] % 2 != 0:
            return None

        decoded = json.loads(payload.decode("utf-8"))
        if not isinstance(decoded, dict):
            raise RuntimeError("Robot state SHM payload must decode to a mapping")
        return decoded

    def _attach(self) -> shared_memory.SharedMemory:
        if self._segment is None:
            self._segment = shared_memory.SharedMemory(name=self.name, create=False)
        return self._segment

    @staticmethod
    def _read_header(segment: shared_memory.SharedMemory) -> tuple[int, int, int, int, int] | None:
        if len(segment.buf) < ROBOT_STATE_HEADER_SIZE:
            return None
        return struct.unpack_from(ROBOT_STATE_HEADER_FMT, segment.buf, 0)
