from __future__ import annotations

import struct
from typing import Protocol


class BufferBackend(Protocol):
    """Common byte-buffer backend interface for CStructShm.

    The backend does not know anything about ctypes.Structure.
    It only stores and retrieves a fixed-size payload byte block.
    """

    payload_size: int

    @property
    def total_size(self) -> int:
        ...

    def clear(self, buf) -> None:
        ...

    def write(self, buf, payload: bytes) -> None:
        ...

    def read_relaxed(self, buf) -> bytes:
        ...

    def read_consistent(self, buf, *, max_retries: int = 10) -> bytes:
        ...


def _check_payload_size(payload: bytes, expected_size: int) -> None:
    if len(payload) != expected_size:
        raise ValueError(
            f"payload size mismatch: got {len(payload)}, expected {expected_size}"
        )


def _check_buffer_size(buf, required_size: int) -> None:
    if len(buf) < required_size:
        raise RuntimeError(
            f"shared memory buffer too small: got {len(buf)}, required {required_size}"
        )


def _clear_region(buf, size: int) -> None:
    buf[:size] = b"\x00" * size


def _read_u32(buf, offset: int) -> int:
    return struct.unpack_from("<I", buf, offset)[0]


def _write_u32(buf, offset: int, value: int) -> None:
    struct.pack_into("<I", buf, offset, int(value) & 0xFFFFFFFF)


def _read_u64(buf, offset: int) -> int:
    return struct.unpack_from("<Q", buf, offset)[0]


def _write_u64(buf, offset: int, value: int) -> None:
    struct.pack_into("<Q", buf, offset, int(value) & 0xFFFFFFFFFFFFFFFF)


def _next_odd_sequence(seq: int) -> int:
    if seq % 2 == 0:
        return (seq + 1) & 0xFFFFFFFFFFFFFFFF
    return (seq + 2) & 0xFFFFFFFFFFFFFFFF


def _next_even_sequence(seq: int) -> int:
    if seq % 2 == 0:
        return seq & 0xFFFFFFFFFFFFFFFF
    return (seq + 1) & 0xFFFFFFFFFFFFFFFF


class PlainBuffer:
    """Single payload buffer without consistency protection.

    Pros:
    - Smallest memory footprint.
    - Fastest and simplest implementation.
    - Compatible with the existing raw shared-memory layout.

    Cons:
    - Does not protect against torn reads.
    - `read_consistent()` is intentionally unsupported.
    - Readers may observe a partially updated payload while a writer is copying.
    """

    def __init__(self, payload_size: int) -> None:
        if payload_size <= 0:
            raise ValueError(f"payload_size must be positive: {payload_size}")
        self.payload_size = int(payload_size)

    @property
    def total_size(self) -> int:
        return self.payload_size

    def clear(self, buf) -> None:
        _check_buffer_size(buf, self.total_size)
        _clear_region(buf, self.total_size)

    def write(self, buf, payload: bytes) -> None:
        _check_buffer_size(buf, self.total_size)
        _check_payload_size(payload, self.payload_size)
        buf[: self.payload_size] = payload

    def read_relaxed(self, buf) -> bytes:
        _check_buffer_size(buf, self.total_size)
        return bytes(buf[: self.payload_size])

    def read_consistent(self, buf, *, max_retries: int = 10) -> bytes:
        raise NotImplementedError(
            "PlainBuffer does not support consistent reads. "
            "Use read_relaxed() or choose SeqLockBuffer / DoubleBuffer."
        )


class SeqLockBuffer:
    """Single payload buffer protected by a sequence counter.

    Memory layout:
        [seq: uint64][payload: payload_size bytes]

    Sequence rule:
        - even seq: stable payload
        - odd seq : writer is updating payload

    Pros:
    - Simple consistency protocol.
    - Small memory overhead: 8 bytes.
    - Good fit for latest-snapshot channels with one writer and many readers.

    Cons:
    - Readers may retry if the writer updates frequently.
    - A stalled writer can leave the sequence odd and block consistent reads.
    - Python/shared-memory writes are not a hard real-time atomicity guarantee;
      this is best treated as an experimental lock-free protocol.
    """

    SEQ_OFFSET = 0
    PAYLOAD_OFFSET = 8

    def __init__(self, payload_size: int) -> None:
        if payload_size <= 0:
            raise ValueError(f"payload_size must be positive: {payload_size}")
        self.payload_size = int(payload_size)

    @property
    def total_size(self) -> int:
        return self.PAYLOAD_OFFSET + self.payload_size

    def clear(self, buf) -> None:
        _check_buffer_size(buf, self.total_size)
        _clear_region(buf, self.total_size)

    def write(self, buf, payload: bytes) -> None:
        _check_buffer_size(buf, self.total_size)
        _check_payload_size(payload, self.payload_size)

        seq = _read_u64(buf, self.SEQ_OFFSET)
        odd_seq = _next_odd_sequence(seq)
        even_seq = _next_even_sequence(odd_seq)

        _write_u64(buf, self.SEQ_OFFSET, odd_seq)
        buf[self.PAYLOAD_OFFSET : self.PAYLOAD_OFFSET + self.payload_size] = payload
        _write_u64(buf, self.SEQ_OFFSET, even_seq)

    def read_relaxed(self, buf) -> bytes:
        _check_buffer_size(buf, self.total_size)
        start = self.PAYLOAD_OFFSET
        end = start + self.payload_size
        return bytes(buf[start:end])

    def read_consistent(self, buf, *, max_retries: int = 10) -> bytes:
        _check_buffer_size(buf, self.total_size)
        if max_retries <= 0:
            raise ValueError(f"max_retries must be positive: {max_retries}")

        start = self.PAYLOAD_OFFSET
        end = start + self.payload_size

        for _ in range(max_retries):
            seq1 = _read_u64(buf, self.SEQ_OFFSET)
            if seq1 % 2 != 0:
                continue

            payload = bytes(buf[start:end])
            seq2 = _read_u64(buf, self.SEQ_OFFSET)

            if seq1 == seq2 and seq2 % 2 == 0:
                return payload

        raise RuntimeError(
            f"SeqLockBuffer failed to read a consistent snapshot "
            f"after {max_retries} retries"
        )


class DoubleBuffer:
    """Two-slot payload buffer with active-slot flipping.

    Memory layout:
        [active_index: uint32][reserved: uint32]
        [slot0_seq: uint64][slot0_payload: payload_size bytes]
        [slot1_seq: uint64][slot1_payload: payload_size bytes]

    Slot sequence rule:
        - even seq: stable slot payload
        - odd seq : writer is updating the slot

    Pros:
    - Writer updates the inactive slot, then flips the active index.
    - Readers usually see a fully written previous or next snapshot.
    - Lower retry pressure than a single seqlock under moderate write rates.

    Cons:
    - Uses roughly 2x payload memory plus header overhead.
    - A very slow reader can still race with slot reuse.
    - Correct writer/reader behavior depends on both sides using this exact layout.
    """

    ACTIVE_INDEX_OFFSET = 0
    RESERVED_OFFSET = 4
    HEADER_SIZE = 8
    SLOT_SEQ_SIZE = 8
    SLOT_COUNT = 2

    def __init__(self, payload_size: int) -> None:
        if payload_size <= 0:
            raise ValueError(f"payload_size must be positive: {payload_size}")
        self.payload_size = int(payload_size)
        self.slot_size = self.SLOT_SEQ_SIZE + self.payload_size

    @property
    def total_size(self) -> int:
        return self.HEADER_SIZE + self.SLOT_COUNT * self.slot_size

    def clear(self, buf) -> None:
        _check_buffer_size(buf, self.total_size)
        _clear_region(buf, self.total_size)
        _write_u32(buf, self.ACTIVE_INDEX_OFFSET, 0)
        _write_u32(buf, self.RESERVED_OFFSET, 0)

    def write(self, buf, payload: bytes) -> None:
        _check_buffer_size(buf, self.total_size)
        _check_payload_size(payload, self.payload_size)

        active_index = self._read_active_index(buf)
        inactive_index = 1 - active_index

        seq_offset = self._slot_seq_offset(inactive_index)
        payload_offset = self._slot_payload_offset(inactive_index)

        seq = _read_u64(buf, seq_offset)
        odd_seq = _next_odd_sequence(seq)
        even_seq = _next_even_sequence(odd_seq)

        _write_u64(buf, seq_offset, odd_seq)
        buf[payload_offset : payload_offset + self.payload_size] = payload
        _write_u64(buf, seq_offset, even_seq)

        _write_u32(buf, self.ACTIVE_INDEX_OFFSET, inactive_index)

    def read_relaxed(self, buf) -> bytes:
        _check_buffer_size(buf, self.total_size)

        active_index = self._read_active_index(buf)
        payload_offset = self._slot_payload_offset(active_index)

        return bytes(buf[payload_offset : payload_offset + self.payload_size])

    def read_consistent(self, buf, *, max_retries: int = 10) -> bytes:
        _check_buffer_size(buf, self.total_size)
        if max_retries <= 0:
            raise ValueError(f"max_retries must be positive: {max_retries}")

        for _ in range(max_retries):
            active_index_1 = self._read_active_index(buf)
            seq_offset = self._slot_seq_offset(active_index_1)
            payload_offset = self._slot_payload_offset(active_index_1)

            seq1 = _read_u64(buf, seq_offset)
            if seq1 % 2 != 0:
                continue

            payload = bytes(buf[payload_offset : payload_offset + self.payload_size])

            seq2 = _read_u64(buf, seq_offset)
            active_index_2 = self._read_active_index(buf)

            if (
                active_index_1 == active_index_2
                and seq1 == seq2
                and seq2 % 2 == 0
            ):
                return payload

        raise RuntimeError(
            f"DoubleBuffer failed to read a consistent snapshot "
            f"after {max_retries} retries"
        )

    def _read_active_index(self, buf) -> int:
        return _read_u32(buf, self.ACTIVE_INDEX_OFFSET) % self.SLOT_COUNT

    def _slot_base_offset(self, slot_index: int) -> int:
        if slot_index not in (0, 1):
            raise ValueError(f"invalid slot index: {slot_index}")
        return self.HEADER_SIZE + slot_index * self.slot_size

    def _slot_seq_offset(self, slot_index: int) -> int:
        return self._slot_base_offset(slot_index)

    def _slot_payload_offset(self, slot_index: int) -> int:
        return self._slot_base_offset(slot_index) + self.SLOT_SEQ_SIZE