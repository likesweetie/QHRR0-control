from __future__ import annotations

import ctypes
import time
from enum import IntEnum
from multiprocessing import shared_memory


class OperatorCommandCode(IntEnum):
    NONE = 0
    ENABLE = 1
    DISABLE = 2
    DAMPING = 3
    ZERO_SET = 4
    ESTOP = 5
    RESET_FAULT = 6


class OperatorCommandC(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("timestamp_ns", ctypes.c_uint64),
        ("command", ctypes.c_uint32),
        ("target_mask", ctypes.c_uint32),
    ]


OPERATOR_COMMAND_SIZE = ctypes.sizeof(OperatorCommandC)


class OperatorCommandShm:
    def __init__(self, name: str, *, create: bool = False, size: int | None = None) -> None:
        self.name = str(name)
        requested_size = OPERATOR_COMMAND_SIZE if size is None else int(size)
        if requested_size < OPERATOR_COMMAND_SIZE:
            raise ValueError(
                f"OperatorCommandShm size is too small: {requested_size}/{OPERATOR_COMMAND_SIZE}"
            )
        self.shm = shared_memory.SharedMemory(
            name=self.name,
            create=bool(create),
            size=requested_size if create else 0,
        )
        if len(self.shm.buf) < OPERATOR_COMMAND_SIZE:
            self.close()
            raise RuntimeError(
                f"OperatorCommandShm segment is too small: {len(self.shm.buf)}/{OPERATOR_COMMAND_SIZE}"
            )
        self._last_timestamp_ns = 0

    @classmethod
    def open_reader(cls, name: str):
        return cls(name, create=False)

    @classmethod
    def open_writer(cls, name: str):
        return cls(name, create=False)

    @classmethod
    def create(cls, name: str, size: int | None = None):
        shm = cls(name, create=True, size=size)
        shm.clear()
        return shm

    def close(self) -> None:
        if self.shm is not None:
            self.shm.close()
            self.shm = None

    def unlink(self) -> None:
        if self.shm is not None:
            self.shm.unlink()

    def clear(self) -> None:
        self.shm.buf[: len(self.shm.buf)] = b"\x00" * len(self.shm.buf)

    def read_relaxed(self) -> OperatorCommandC:
        return OperatorCommandC.from_buffer_copy(self.shm.buf[:OPERATOR_COMMAND_SIZE])

    def read_new(self) -> OperatorCommandC | None:
        command = self.read_relaxed()
        if command.timestamp_ns == 0 or command.timestamp_ns == self._last_timestamp_ns:
            return None
        self._last_timestamp_ns = int(command.timestamp_ns)
        return command

    def write(self, command: OperatorCommandC) -> None:
        data = bytes(command)
        self.shm.buf[: len(data)] = data

    def publish(self, code: OperatorCommandCode | int, target_mask: int = 0) -> int:
        command = OperatorCommandC()
        command.timestamp_ns = time.time_ns()
        command.command = int(code)
        command.target_mask = int(target_mask)
        self.write(command)
        return int(command.timestamp_ns)

